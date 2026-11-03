#!/usr/bin/env python3
"""
Close one finger smoothly until it feels a target pressure, then stop and
report what the sensor reads.

Usage:
    python3 close_until_pressure.py <hand> <finger> <grams>
    python3 close_until_pressure.py left index 200

hand   = left or right
finger = thumb, index, middle, ring, or pinky
grams  = the pressure you want, written as a weight in grams

Every run also writes a CSV log (test_log_<hand>_<finger>_<grams>g_<time>.csv)
with one row per sensor message: time, phase, commanded position, actual
position, the three normal forces, and the force used (raw and in Newtons).
"""
import csv
import statistics
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from ros2_stark_interfaces.msg import TouchStatus, MotorStatus

# ---------------- SETTINGS (change these to match your setup) ----------------
TOUCH_TOPICS   = {"left": "/brainco/touch_status", "right": "/brainco/touch_status_r"}
MOTOR_TOPICS   = {"left": "/brainco/motor_status", "right": "/brainco/motor_status_r"}
COMMAND_TOPICS = {"left": "/brainco/cmd_left",     "right": "/brainco/cmd_right"}

# Which spot in the touch message each finger is.
# CHECK THIS: hands_controller.cpp calls data[0] the "index finger", but this
# assumes thumb is first. Press one fingertip and see which spot reacts.
TOUCH_INDEX = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

# The 6 motor positions, in the order the BrainCo Stark uses them (only used for messages)
JOINT_NAMES = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]

# Which of the 6 motor positions closes each finger
# (the 6 are: thumb, thumb_aux, index, middle, ring, pinky)
JOINT_INDEX = {"thumb": 0, "index": 2, "middle": 3, "ring": 4, "pinky": 5}

# All the joints that belong to the finger under test. These are NOT pre-closed.
# (The thumb has two joints, so testing the thumb leaves both alone.)
TEST_JOINTS = {"thumb": [0, 1], "index": [2], "middle": [3], "ring": [4], "pinky": [5]}

# The joint that closes LAST, after THUMB_DELAY. Your gesture code (pre_fist =
# 0,1,1,1,1,1) closes thumb_aux together with the fingers and the main thumb
# joint afterward, so only joint 0 is delayed here.
THUMB_LAST = [0]

OPEN_POS    = 0.0    # joint value when the finger is fully open
THUMB_DELAY = 0.5   # seconds to wait between closing the fingers and closing the thumb

CLOSED_POS = 1.0     # joint value when the finger is fully closed (0.0 is open)
STEP       = 0.005   # how far the finger moves each tick (smaller = slower, smoother)
RATE_HZ    = 50      # ticks per second
SPEED      = 0.5     # speed sent to the hand (0 to 1). Lower = gentler.

# Multiply the sensor reading by this to get Newtons.
# 0.01 means the sensor reports hundredths of a Newton (raw / 100 = Newtons),
# so the maximum reading of 2500 raw = 25 N. A 100 g weight should read about 98 raw.
SENSOR_TO_NEWTONS = 0.01

SENSOR_MAX_RAW = 2500   # the reading you saw when squeezing as hard as possible (sensor maxed out)
MAX_NEWTONS = 25.0      # the sensor's top limit; the program refuses to go past it
# -----------------------------------------------------------------------------


def main():
    if len(sys.argv) != 4 or sys.argv[1] not in TOUCH_TOPICS or sys.argv[2] not in TOUCH_INDEX:
        print("Usage: python3 close_until_pressure.py <left|right> <thumb|index|middle|ring|pinky> <grams>")
        return

    hand = sys.argv[1]
    finger = sys.argv[2]
    grams = float(sys.argv[3])
    target_n = grams / 1000 * 9.81          # grams -> kilograms -> Newtons
    joint = JOINT_INDEX[finger]

    if target_n > MAX_NEWTONS:
        print(f"{grams} g is {target_n:.1f} N, above the {MAX_NEWTONS} N limit. Pick less.")
        return

    rclpy.init()
    node = rclpy.create_node("close_until_pressure")

    latest = {"recent": [], "touch_count": 0, "motors": None}

    # --- Data logger: one CSV row every time a touch message streams in ---
    log_state = {"phase": "startup", "cmd": ""}   # phase and last commanded position of our finger
    start_time = time.time()
    log_name = f"test_log_{hand}_{finger}_{grams:g}g_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_name, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["time_s", "phase", "commanded_position", "actual_position",
                  "normal_force1", "normal_force2", "normal_force3",
                  "raw_force_used", "force_N"])

    def on_touch(msg):
        item = msg.data[TOUCH_INDEX[finger]]
        # the sensor has 3 normal-force values per finger; use the biggest one
        forces = [item.normal_force1, item.normal_force2, item.normal_force3]
        raw = float(max(forces))
        latest["recent"] = (latest["recent"] + [raw])[-5:]   # keep the last 5 readings
        latest["touch_count"] += 1

        actual = round(latest["motors"][joint], 3) if latest["motors"] else ""
        log.writerow([round(time.time() - start_time, 3), log_state["phase"],
                      log_state["cmd"], actual, *forces,
                      raw, round(raw * SENSOR_TO_NEWTONS, 3)])
        log_file.flush()                                     # so nothing is lost if you press Ctrl+C

    def smooth_raw():
        """Median of the last 5 readings, so one noisy spike can't trigger the stop."""
        return statistics.median(latest["recent"]) if latest["recent"] else 0.0

    def on_motors(msg):
        latest["motors"] = [float(p) for p in msg.positions]   # plain floats, ROS requires it

    node.create_subscription(TouchStatus, TOUCH_TOPICS[hand], on_touch, 10)
    node.create_subscription(MotorStatus, MOTOR_TOPICS[hand], on_motors, 10)
    pub = node.create_publisher(JointState, COMMAND_TOPICS[hand], 10)

    def finish():
        log_file.close()
        node.destroy_node()
        rclpy.shutdown()

    # --- Step 1: make sure the hand is actually connected before moving ---
    print("Waiting for the hand...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (latest["touch_count"] > 0 and latest["motors"] is not None
                and pub.get_subscription_count() > 0):
            break

    if latest["touch_count"] == 0 or latest["motors"] is None:
        print(f"ERROR: no data from {TOUCH_TOPICS[hand]} / {MOTOR_TOPICS[hand]}. "
              "Is the hand running, and does ROS_DOMAIN_ID match the robot?")
        finish()
        return
    if pub.get_subscription_count() == 0:
        print(f"ERROR: nothing is listening on {COMMAND_TOPICS[hand]}.")
        finish()
        return

    # --- Step 2: remember where every finger is right now ---
    hold = [float(p) for p in latest["motors"]]   # all 6 joints stay here, except our finger
    hold[joint] = OPEN_POS                        # the finger under test starts fully open

    def move_finger(position):
        msg = JointState()
        positions = list(hold)
        positions[joint] = position
        msg.position = positions
        msg.velocity = [SPEED] * len(positions)   # the hand needs this, like your gesture code
        pub.publish(msg)
        log_state["cmd"] = round(position, 3)     # remember what we commanded, for the log

    # --- Step 3: close every OTHER finger fully, then wait 1 second ---
    def wait_while_holding(seconds):
        end = time.time() + seconds
        while time.time() < end:
            move_finger(hold[joint])        # keep sending the current hold positions
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(1 / RATE_HZ)

    def close_joints(joints):
        """Command these joints fully closed and wait (up to 5 s) until they get there."""
        for j in joints:
            hold[j] = CLOSED_POS
        deadline = time.time() + 5.0
        while time.time() < deadline:
            move_finger(hold[joint])
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(abs(latest["motors"][j] - CLOSED_POS) < 0.05 for j in joints):
                return True
            time.sleep(1 / RATE_HZ)
        return False

    others = [j for j in range(len(hold)) if j not in TEST_JOINTS[finger]]
    thumb_joints = [j for j in others if j in THUMB_LAST]    # empty when testing the thumb
    other_fingers = [j for j in others if j not in thumb_joints]   # includes thumb_aux

    log_state["phase"] = "prep"
    print("Closing the other fingers...")
    ok = close_joints(other_fingers)

    if thumb_joints:                        # the thumb closes LAST, after a delay
        print(f"Waiting {THUMB_DELAY} s before closing the thumb...")
        wait_while_holding(THUMB_DELAY)
        print("Closing the thumb...")
        ok = close_joints(thumb_joints) and ok

    if not ok:
        stuck = {JOINT_NAMES[j]: round(latest["motors"][j], 2) for j in others
                 if abs(latest["motors"][j] - CLOSED_POS) >= 0.05}
        print(f"WARNING: these joints did not fully close within 5 seconds: {stuck}. Continuing anyway.")

    # make sure the finger under test has actually reached fully open
    deadline = time.time() + 5.0
    while time.time() < deadline and abs(latest["motors"][joint] - OPEN_POS) > 0.05:
        wait_while_holding(0.1)
    if abs(latest["motors"][joint] - OPEN_POS) > 0.05:
        print(f"WARNING: the {finger} did not fully open within 5 seconds. Continuing anyway.")

    print("Waiting 1 second...")
    wait_while_holding(1.0)

    # --- Step 4: record the resting sensor reading so we only count NEW pressure ---
    log_state["phase"] = "baseline"
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    baseline = smooth_raw()
    print(f"Resting sensor reading (subtracted from everything below): {baseline:.1f} raw")

    def pressure():
        """New pressure since the baseline, in RAW sensor units."""
        return smooth_raw() - baseline

    target_raw = target_n / SENSOR_TO_NEWTONS
    if SENSOR_TO_NEWTONS == 1.0:
        print("WARNING: sensor scale not calibrated (SENSOR_TO_NEWTONS = 1.0). "
              f"Your target is being compared as {target_raw:.2f} RAW units, which is tiny.")

    # --- Step 5: close the finger under test a little at a time ---
    print(f"Closing {hand} {finger} until {grams} g ({target_n:.2f} N)...")

    log_state["phase"] = "test"
    position = hold[joint]                  # start from where the finger is now
    reached = False
    last_print = 0.0
    while rclpy.ok() and position < CLOSED_POS:
        rclpy.spin_once(node, timeout_sec=0)        # read the latest sensor value
        if pressure() >= target_raw:
            reached = True
            break
        position = min(position + STEP, CLOSED_POS) # small step = smooth motion
        move_finger(position)
        if time.time() - last_print > 0.5:          # progress update twice a second
            print(f"  position {position:.2f}   pressure {pressure():.1f} raw")
            last_print = time.time()
        time.sleep(1 / RATE_HZ)

    # STOP: keep holding the last position we COMMANDED. Do not send the finger's
    # reported position back to it: that reading lags behind the command (and the
    # object is holding the finger back), so sending it makes the finger jerk backward.
    log_state["phase"] = "hold"
    hold[joint] = position

    # let the reading settle for a second, then take the middle value of what we saw
    samples = []
    end = time.time() + 1.0
    while time.time() < end:
        move_finger(position)               # same command again, so the finger stays put
        rclpy.spin_once(node, timeout_sec=0.05)
        samples.append(pressure())
        time.sleep(1 / RATE_HZ)

    final_raw = statistics.median(samples)
    final_n = final_raw * SENSOR_TO_NEWTONS
    print("Reached target pressure." if reached else "Finger fully closed without reaching the target.")
    print(f"Final reading: {final_raw:.1f} raw  =  {final_n:.2f} N  ({final_n / 9.81 * 1000:.0f} g)")
    print(f"(range during the last second: {min(samples):.1f} to {max(samples):.1f} raw)")
    if max(samples) + baseline >= SENSOR_MAX_RAW:
        print(f"NOTE: the sensor hit its maximum ({SENSOR_MAX_RAW:.0f} raw). The real pressure may be higher.")
    print(f"Data saved to {log_name}")

    finish()


if __name__ == "__main__":
    main()
