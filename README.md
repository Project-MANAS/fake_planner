# Fake Planner

ROS package for basic navigation to a goal without obstacle avoidance.

### Usage
Set the following parameters in `fake_planner.launch`:
  - `goal_topic (default: "move_base_simple/goal")`: Topic where goals are published (type: *geometry_msgs/PoseStamped*)
  - `cmd_vel_topic (default: "/cmd_vel")`: Topic where velocity commands should be published
  - `max_velocity_service (default: "set_max_velocity")`: Name where service should be advertised to set maximum travel velocity
  - `linear_tolerance (default: 0.1)`: Tolerance for linear displacement between the bot and the goal (m)
  - `angular_tolerance (default: 0.1)`: Tolerance for angular displacement between the bot and the goal (rad)
  - `time_to_x (default: 0.5)`: Time it would take to reach the goal if the max velocity permits. Used for scaling linear velocity.
  - `time_to_angle (default: 1.0)`: Time it would take to reach the goal if the max velocity permits. Used for scaling angular velocity.
  - `max_linear_speed (default: 1.0)`: Maximum linear speed (m/s)
  - `max_angular_speed (default: 0.5)`: Maximum angular speed (rad/s)

To launch the fake planner node:
```bash
roslaunch fake_planner fake_planner.launch
```
