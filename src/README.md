# Roboost Cortex - Packages

Based on

- [Articulated Robotics](https://www.youtube.com/watch?v=CwdbsvcpOHM&ab_channel=ArticulatedRobotics)
- [Mobile Robot URDF blogpost](https://medium.com/teamarimac/create-a-mobile-robot-model-with-ros-urdf-4dc46446db7f)
- [ROS2 Docs](https://docs.ros.org/en/humble/index.html)

## Installation

To install the packages, run the following command in the root of the workspace:

```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

Then source the workspace:

```bash
source install/local_setup.bash
```

## Packages

### tf_broadcast_package

This package is used to broadcast the tf tree of the robot. This includes the URDF file and the odom frame in the case of the Roboost Mecanum robot.

#### Usage

To launch the package, run the following command:

```bash
ros2 launch tf_broadcast_package mecanum_tf_broadcast_launch.py
```
