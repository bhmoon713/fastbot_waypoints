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

#for passing result
## and run below test commands(Terminal 3)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

#for failing result
Edit file "test_fastbot_waypoint.cpp"

Line 128 
EXPECT_GT(dist, 0.05) -- > EXPECT_GT(dist, 10.0)

## and run below test commands(Terminal 3)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all