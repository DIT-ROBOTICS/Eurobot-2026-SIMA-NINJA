ros2 launch ninja-sima-main ninja-sima-bringup.launch.py team:=blue

ros2 topic pub --once /robot/startup/ninja/start std_msgs/msg/Int16 "{data: 1}"