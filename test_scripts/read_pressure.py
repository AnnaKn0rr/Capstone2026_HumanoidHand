#!/usr/bin/env python3
"""
Show one finger's pressure reading live, and log it to a CSV.
This script only LISTENS. It never moves the hand.

Usage:
    python3 read_pressure.py <hand> <finger>
    python3 read_pressure.py left index

hand   = left or right
finger = thumb, index, middle, ring, or pinky

Press Ctrl+C to stop. Every run writes a NEW CSV into the LOG_FOLDER setting
below, named <hand>_<finger>_reading_<date>_<time>.csv  (for example
left_index_reading_2026-11-08_14-31-20.csv). One row per sensor message:
time, actual position, the three normal forces, the force used (raw, N and g).
"""
import csv
import os
import sys
import time

import rclpy
from ros2_stark_interfaces.msg import TouchStatus, MotorStatus

# ---------------- SETTINGS ----------------
TOUCH_TOPICS = {"left": "/brainco/touch_status", "right": "/brainco/touch_status_r"}
MOTOR_TOPICS = {"left": "/brainco/motor_status", "right": "/brainco/motor_status_r"}

LOG_FOLDER = "~/UNCW-G1-Humanoid-Robot/src/g1_gestures/src/Capstone2026_HumanoidHand/test_scripts/logger_data"

# Which spot in the touch message each finger is (unverified, see close_until_pressure.py)
TOUCH_INDEX = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}

# Which of the 6 motor positions belongs to each finger (thumb, thumb_aux, index, middle, ring, pinky)
JOINT_INDEX = {"thumb": 0, "index": 2, "middle": 3, "ring": 4, "pinky": 5}

SENSOR_TO_NEWTONS = 0.01   # raw / 100 = Newtons
SENSOR_MAX_RAW = 2500      # the sensor's top reading (25 N)
# -----------------------------------------------------------------------------


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in TOUCH_TOPICS or sys.argv[2] not in TOUCH_INDEX:
        print("Usage: python3 read_pressure.py <left|right> <thumb|index|middle|ring|pinky>")
        return

    hand, finger = sys.argv[1], sys.argv[2]
    joint = JOINT_INDEX[finger]

    # --- CSV logger ---
    folder = os.path.expanduser(LOG_FOLDER)
    if not os.path.isdir(folder):
        print(f"NOTE: {folder} did not exist, so it was created. "
              "If you expected an existing logger_data folder, check LOG_FOLDER at the top of this file.")
    os.makedirs(folder, exist_ok=True)
    log_name = os.path.join(
        folder, f"{hand}_{finger}_reading_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv")
    log_file = open(log_name, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["time_s", "actual_position", "normal_force1", "normal_force2",
                  "normal_force3", "raw_force_used", "force_N", "force_g"])

    rclpy.init()
    node = rclpy.create_node("read_pressure")
    state = {"motors": None, "count": 0, "peak": 0.0}
    start_time = time.time()

    def on_motors(msg):
        state["motors"] = [float(p) for p in msg.positions]

    def on_touch(msg):
        item = msg.data[TOUCH_INDEX[finger]]
        forces = [item.normal_force1, item.normal_force2, item.normal_force3]
        raw = float(max(forces))                 # the sensor has 3 values per finger; use the biggest
        newtons = raw * SENSOR_TO_NEWTONS
        grams = newtons / 9.81 * 1000
        actual = round(state["motors"][joint], 3) if state["motors"] else ""

        state["count"] += 1
        state["peak"] = max(state["peak"], raw)

        log.writerow([round(time.time() - start_time, 3), actual, *forces,
                      raw, round(newtons, 3), round(grams, 1)])
        log_file.flush()                          # so nothing is lost if you press Ctrl+C

        # live display: one line that keeps updating in place
        print(f"\r{raw:7.0f} raw   {newtons:6.2f} N   {grams:6.0f} g   "
              f"(peak {state['peak']:.0f} raw)   forces {forces}   ", end="", flush=True)

    node.create_subscription(TouchStatus, TOUCH_TOPICS[hand], on_touch, 10)
    node.create_subscription(MotorStatus, MOTOR_TOPICS[hand], on_motors, 10)

    print(f"Reading the {hand} {finger} finger. Press Ctrl+C to stop.\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("\n")
    if state["count"] == 0:
        print(f"No data arrived on {TOUCH_TOPICS[hand]}. Check the robot is on and ROS_DOMAIN_ID matches.")
    else:
        peak_n = state["peak"] * SENSOR_TO_NEWTONS
        print(f"Peak reading: {state['peak']:.0f} raw = {peak_n:.2f} N ({peak_n / 9.81 * 1000:.0f} g)")
        if state["peak"] >= SENSOR_MAX_RAW:
            print(f"NOTE: the sensor hit its maximum ({SENSOR_MAX_RAW} raw). The real pressure may be higher.")
        print(f"{state['count']} readings saved to {os.path.abspath(log_name)}")

    log_file.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
