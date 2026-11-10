#!/usr/bin/env python3
"""
Close the WHOLE hand smoothly until every fingertip feels a target pressure,
then report what each fingertip reads.

This works like close_until_pressure.py, but all five fingers close together.
Each finger stops on its own as soon as ITS sensor reaches the target, and the
fingers that have not reached it yet keep closing.

Usage:
    python3 close_hand_until_pressure.py <hand> <grams>
    python3 close_hand_until_pressure.py left 100

hand  = left or right
grams = the pressure you want on EACH fingertip, written as a weight in grams

Every run writes a NEW CSV log into the LOG_FOLDER setting below, named
<hand>_hand_<grams>g_<date>_<time>.csv  (for example
left_hand_100g_2026-11-08_17-30-12.csv). It has one row per sensor message:
the time, the phase, and for each finger its commanded position, actual
position, raw force, force in grams, and whether it has stopped.
"""
import csv
import os
import statistics
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from ros2_stark_interfaces.msg import TouchStatus, MotorStatus

# ---------------- SETTINGS ----------------
TOUCH_TOPICS   = {"left": "/brainco/touch_status", "right": "/brainco/touch_status_r"}
MOTOR_TOPICS   = {"left": "/brainco/motor_status", "right": "/brainco/motor_status_r"}
COMMAND_TOPICS = {"left": "/brainco/cmd_left",     "right": "/brainco/cmd_right"}

# Where the test results are saved
LOG_FOLDER = "~/UNCW-G1-Humanoid-Robot/src/g1_gestures/src/Capstone2026_HumanoidHand/test_scripts/logger_data"

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# Which spot in the touch message each finger is 
TOUCH_INDEX = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

# The 6 motor positions are: thumb, thumb_aux, index, middle, ring, pinky
JOINT_NAMES = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]
JOINT_INDEX = {"thumb": 0, "index": 2, "middle": 3, "ring": 4, "pinky": 5}

# The thumb has a second joint (thumb_aux) that swings it across the palm. It is
# moved to this position during setup and then left alone; only the main thumb
# joint closes during the test. 1.0 matches your gesture code (pre_fist).
THUMB_AUX = 1
THUMB_AUX_POS = 1.0

OPEN_POS   = 0.0     # joint value when a finger is fully open
CLOSED_POS = 1.0     # joint value when a finger is fully closed
STEP       = 0.001   # how far each finger moves per tick (the smallest step the hand can resolve)
RATE_HZ    = 50      # ticks per second
SPEED      = 0.2     # speed sent to the hand DURING THE TEST (0 to 1)
PREP_SPEED = 0.5     # speed for the setup move (opening the hand)

# Multiply the sensor reading by this to get Newtons (raw / 100 = N, so 2500 raw = 25 N)
SENSOR_TO_NEWTONS = 0.01
SENSOR_MAX_RAW = 2500
MAX_NEWTONS = 25.0      # the sensor's top limit; the program refuses to go past it
# -----------------------------------------------------------------------------


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in TOUCH_TOPICS:
        print("Usage: python3 close_hand_until_pressure.py <left|right> <grams>")
        return

    hand = sys.argv[1]
    grams = float(sys.argv[2])
    target_n = grams / 1000 * 9.81          # grams -> kilograms -> Newtons

    if target_n > MAX_NEWTONS:
        print(f"{grams} g is {target_n:.1f} N, above the {MAX_NEWTONS} N limit. Pick less.")
        return

    rclpy.init()
    node = rclpy.create_node("close_hand_until_pressure")

    # one list of recent readings per finger, so a single noisy spike can't trigger a stop
    latest = {"recent": {f: [] for f in FINGERS}, "touch_count": 0, "motors": None}

    # --- Data logger: one CSV row every time a touch message streams in ---
    log_state = {"phase": "startup",
                 "cmd": {f: "" for f in FINGERS},         # last commanded position of each finger
                 "stopped": {f: 0 for f in FINGERS}}      # 1 once a finger has stopped closing
    start_time = time.time()

    folder = os.path.expanduser(LOG_FOLDER)
    if not os.path.isdir(folder):
        print(f"NOTE: {folder} did not exist, so it was created. "
              "If you expected an existing logger_data folder, check LOG_FOLDER at the top of this file.")
    os.makedirs(folder, exist_ok=True)
    log_name = os.path.join(
        folder, f"{hand}_hand_{grams:g}g_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv")
    log_file = open(log_name, "w", newline="")
    log = csv.writer(log_file)
    header = ["time_s", "phase"]
    for f in FINGERS:
        header += [f"{f}_commanded", f"{f}_actual", f"{f}_raw", f"{f}_g", f"{f}_stopped"]
    header.append("scale_g")
    log.writerow(header)

    def on_touch(msg):
        row = [round(time.time() - start_time, 3), log_state["phase"]]
        for f in FINGERS:
            item = msg.data[TOUCH_INDEX[f]]
            # the sensor has 3 normal-force values per finger; use the biggest one
            raw = float(max(item.normal_force1, item.normal_force2, item.normal_force3))
            latest["recent"][f] = (latest["recent"][f] + [raw])[-5:]   # keep the last 5 readings
            actual = round(latest["motors"][JOINT_INDEX[f]], 3) if latest["motors"] else ""
            grams_now = raw * SENSOR_TO_NEWTONS / 9.81 * 1000
            row += [log_state["cmd"][f], actual, raw, round(grams_now, 1), log_state["stopped"][f]]
        latest["touch_count"] += 1
        log.writerow(row + [""])
        log_file.flush()                                  # so nothing is lost if press Ctrl+C

    def smooth_raw(f):
        """Median of the last 5 readings for one finger."""
        r = latest["recent"][f]
        return statistics.median(r) if r else 0.0

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

    # --- Step 2: the command we keep sending. It holds all 6 joints. ---
    hold = [float(p) for p in latest["motors"]]
    speed = {"now": PREP_SPEED}                   # fast for setup, switched to SPEED for the test

    def send():
        msg = JointState()
        msg.position = list(hold)
        msg.velocity = [speed["now"]] * len(hold)   # the hand needs this, like your gesture code
        pub.publish(msg)
        for f in FINGERS:
            log_state["cmd"][f] = round(hold[JOINT_INDEX[f]], 3)

    def wait_while_holding(seconds):
        end = time.time() + seconds
        while time.time() < end:
            send()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(1 / RATE_HZ)

    # --- Step 3: open every finger fully (and set the thumb's swing joint), then wait 1 second ---
    log_state["phase"] = "prep"
    print("Opening the hand...")
    for f in FINGERS:
        hold[JOINT_INDEX[f]] = OPEN_POS
    hold[THUMB_AUX] = THUMB_AUX_POS

    deadline = time.time() + 5.0
    while time.time() < deadline:
        wait_while_holding(0.1)
        if all(abs(latest["motors"][j] - hold[j]) < 0.05 for j in range(len(hold))):
            break
    stuck = {JOINT_NAMES[j]: round(latest["motors"][j], 2) for j in range(len(hold))
             if abs(latest["motors"][j] - hold[j]) >= 0.05}
    if stuck:
        print(f"WARNING: these joints did not reach their start position within 5 seconds: {stuck}. Continuing anyway.")

    print("Waiting 1 second...")
    wait_while_holding(1.0)

    # --- Step 4: record each finger's resting reading so we only count NEW pressure ---
    log_state["phase"] = "baseline"
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    baseline = {f: smooth_raw(f) for f in FINGERS}
    print("Resting readings (subtracted from everything below): "
          + ", ".join(f"{f} {baseline[f]:.0f}" for f in FINGERS) + " raw")

    def pressure(f):
        """New pressure on one finger since the baseline, in RAW sensor units."""
        return smooth_raw(f) - baseline[f]

    target_raw = target_n / SENSOR_TO_NEWTONS

    # --- Step 5: close all fingers a little at a time; each one stops at the target ---
    print(f"Closing the {hand} hand until each fingertip reads {grams:g} g ({target_n:.2f} N)...")

    log_state["phase"] = "test"
    speed["now"] = SPEED                              # slow from here on
    position = {f: hold[JOINT_INDEX[f]] for f in FINGERS}
    done = {f: False for f in FINGERS}                # True once a finger has stopped
    reached = {f: False for f in FINGERS}             # True if it stopped because of pressure
    last_print = 0.0
    while rclpy.ok() and not all(done.values()):
        rclpy.spin_once(node, timeout_sec=0)          # read the latest sensor values
        for f in FINGERS:
            if done[f]:
                continue                              # this finger stays where it stopped
            if pressure(f) >= target_raw:
                done[f], reached[f] = True, True
                log_state["stopped"][f] = 1
                print(f"  {f} reached the target at position {position[f]:.3f}")
                continue
            position[f] = min(position[f] + STEP, CLOSED_POS)   # small step = smooth motion
            hold[JOINT_INDEX[f]] = position[f]
            if position[f] >= CLOSED_POS:
                done[f] = True
                log_state["stopped"][f] = 1
                print(f"  {f} fully closed without reaching the target")
        send()
        if time.time() - last_print > 0.5:            # progress update twice a second
            print("  " + "  ".join(
                f"{f} {position[f]:.2f}/{pressure(f) * SENSOR_TO_NEWTONS / 9.81 * 1000:.0f}g"
                for f in FINGERS))
            last_print = time.time()
        time.sleep(1 / RATE_HZ)

    # STOP: every finger keeps holding the last position we COMMANDED. We never send a
    # finger's reported position back to it: that reading lags behind the command, so
    # sending it makes the finger jerk backward.
    log_state["phase"] = "hold"
    samples = {f: [] for f in FINGERS}
    end = time.time() + 1.0
    while time.time() < end:
        send()                                        # same command again, so nothing moves
        rclpy.spin_once(node, timeout_sec=0.05)
        for f in FINGERS:
            samples[f].append(pressure(f))
        time.sleep(1 / RATE_HZ)

    print("\nFinal readings:")
    total_g = 0.0
    for f in FINGERS:
        raw = statistics.median(samples[f])
        n = raw * SENSOR_TO_NEWTONS
        g = n / 9.81 * 1000
        total_g += g
        status = "reached target" if reached[f] else "closed fully, target NOT reached"
        note = "   (sensor at its maximum, real pressure may be higher)" if max(samples[f]) + baseline[f] >= SENSOR_MAX_RAW else ""
        print(f"  {f:7s} {raw:7.1f} raw = {n:5.2f} N = {g:6.0f} g   {status}{note}")

    # --- Ask for the scale reading (the fingers keep pressing while you read it) ---
    scale_g = ""
    try:
        answer = input("\nWhat does the scale read, in grams? (press Enter to skip): ").strip()
        if answer:
            scale_g = float(answer)
    except ValueError:
        print("That wasn't a number, so no scale reading was saved.")
    except (EOFError, KeyboardInterrupt):
        print("\nNo scale reading entered.")
    if scale_g != "":
        print(f"Scale {scale_g:g} g   (the five fingertip readings add up to {total_g:.0f} g)")
        # one last row in the CSV: phase "scale", with the scale reading in the last column
        row = [round(time.time() - start_time, 3), "scale"] + [""] * (5 * len(FINGERS)) + [scale_g]
        log.writerow(row)

    print(f"Data saved to {os.path.abspath(log_name)}")

    finish()


if __name__ == "__main__":
    main()
