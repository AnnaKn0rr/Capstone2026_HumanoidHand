Source workspace:
  source /opt/ros/humble/setup.bash
  source <your_workspace>/install/setup.bash

Check hand is visible:
  ros2 topic list | grep brainco
  ros2 topic echo /brainco/touch_status


close_until_pressure.py:

Closes one finger smoothly until a target pressure (in grams) is reached, 
prints the final sensor reading, asks for a manual scale reading (grams), 
and logs a CSV to logger_data/ (e.g. left_index_200g_<date>_<time>.csv).


  python3 close_until_pressure.py  (add arguments: hand, finger, target grams)
for example:   python3 close_until_pressure.py  left index 100

Uses /brainco/cmd_left, /brainco/touch_status, /brainco/motor_status. Raw force / 100 = Newtons (sensor max 2500 raw = 25 N).


stark_csv_logger.py:

Logs every TouchStatus message (all 5 fingers) to CSV.

python3 stark_csv_logger.py
defaults: touch_data.csv, /touch_status
python3 stark_csv_logger.py <out.csv> <topic>
