
# Ros1 Implementierung

- Install mit catkin_make: https://github.com/ros-drivers/driver_common.git
- Install mit catkin_make: https://github.com/ros-drivers/hokuyo_node.git
- source /workspace/devel/setup.zsh (Shell-Abhängig)

- Wenn nicht ttyACM0: `rosparam set hokuyo_node/port /dev/ttyACMx`

- rosrun hokuyo_node hokuyo_node
