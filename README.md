# fastbot_waypoints — Testing Instructions (ROS 2 + GTest)

## Start simulation (Terminal 1)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```

## Launch the Waypoints Action Server for ROS 2 (Terminal 2)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

## Launch the Waypoints Action Server for ROS 2 (Terminal 2)
For passing condition, please make change on below line in "tortoisebot_waypoints waypoints_test.test"
```bash
    declare_parameter<double>("expected_x", 1.0);
    declare_parameter<double>("expected_y", 0.0);
    declare_parameter<double>("expected_yaw_deg", 0.0);
```
Fail condition, please make change on below line in "tortoisebot_waypoints waypoints_test.test"
```bash
    declare_parameter<double>("expected_x", 2.0);
    declare_parameter<double>("expected_y", 1.0);
    declare_parameter<double>("expected_yaw_deg", 10.0);
```
and run below commands(Terminal 3)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```