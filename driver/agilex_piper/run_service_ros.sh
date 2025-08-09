cp src/setup.py src/piper_ros/src/piper/setup.py
colcon build
source install/setup.bash
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
