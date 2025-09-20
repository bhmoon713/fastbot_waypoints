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

## Passing/Failing test
we are going to pass test by moving within safe range within living room and fail by sending robot out of floor.
at the simulation start, robot start at this position
```bash
pose:
  pose:
    position:
      x: 1.497041043145493
      y: 1.2551360548561186
      z: 0.2168336231258006
``` 
### for passing result
Edit file "test_fastbot_waypoint.cpp"
LINE112 
```bash
  // Define the goal
  Waypoint::Goal goal;
  goal.position.x = 2.0;
  goal.position.y = 2.0;
  double goal_yaw = 1.57;
``` 
### and run below test commands(Terminal 3)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

### for failing result
Edit file "test_fastbot_waypoint.cpp"

strike out LINE118.
```bash
  // Define the goal for fail
  Waypoint::Goal goal;
    goal.position.x = -1.0;
    goal.position.y = 0.0;
    double goal_yaw = 1.57;
```
This will move robot to middle of living room.

### and run below test commands(Terminal 3)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```
This will move robot out of floor, so fall down, so that the robot wheel moving but robot cannot move forward, this will make final target position not reached

