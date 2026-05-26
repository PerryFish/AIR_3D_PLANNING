# Reference Repository Strategy

The current MVP does not vendor or compile any external GitHub repository. This keeps the workspace reproducible on Ubuntu 22.04 and ROS2 Humble.

## Reviewed Directions

| Project / Direction | Status | Decision |
| --- | --- | --- |
| PX4 `px4_ros_com` offboard examples | ROS2 direction, Humble is the recommended ROS2 platform in PX4 docs | Useful for future PX4 bridge, not required for this MVP |
| ARK-Electronics ROS2 PX4 offboard example | ROS2 Humble example | Useful reference for velocity/setpoint control, not vendored |
| Fast-Planner | Historically ROS1/Kinetic/Melodic-focused | Not used as the main workspace because direct ROS2 Humble integration is risky |
| EGO-Planner / EGO-Planner-Swarm | Strong quadrotor local planning reference, mostly ROS1-oriented upstream | Not used for MVP to avoid dependency and porting risk |
| FUEL | Exploration planner research stack, ROS1-heavy dependencies | Not used for MVP |
| OctoMap / Voxblox / ESDF planners | Good future mapping layer | Not used yet because MVP uses deterministic obstacle boxes |

## Sources Checked

- PX4 ROS2 offboard example documentation: https://docs.px4.io/main/en/ros2/offboard_control.html
- PX4 ROS2 platform notes: https://docs.px4.io/v1.14/zh/ros/ros2_offboard_control
- ARK ROS2 PX4 offboard example: https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example
- PX4 `px4_ros_com` example source: https://github.com/PX4/px4_ros_com
- EGO-Planner paper/reference: https://arxiv.org/abs/2008.08835

## Current Policy

The main branch should stay locally buildable. External planners can be evaluated in separate branches or packages only after their ROS2 Humble compatibility and dependencies are proven.
