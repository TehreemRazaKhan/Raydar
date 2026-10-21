# raydar

Custom radar sensor plugin for Gazebo Garden + ROS 2 Humble.

Publishes radar target detections on `/radar/targets` and RViz markers on `/radar/markers`.

## Requirements

- ROS 2 Humble
- Gazebo Garden (`gz-sim7`, `gz-rendering7`)

## Build

```bash
colcon build --merge-install --packages-select raydar
source install/setup.bash
