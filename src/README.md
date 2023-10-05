# Roboost Cortex - Packages

Based on

- [Articulated Robotics](https://www.youtube.com/watch?v=CwdbsvcpOHM&ab_channel=ArticulatedRobotics)
- [Mobile Robot URDF blogpost](https://medium.com/teamarimac/create-a-mobile-robot-model-with-ros-urdf-4dc46446db7f)

## Robot state publisher

The robot_state_publisher is used to publish the state of a robot to tf2. It listens to the joint states and calculates the resulting transformations and publishes them to tf2. Also, it publishes the robot model in /robot_description.

It will eventually be executed as a node in the robot's launch file, but for now it can be executed as a standalone node with the following command:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro ./urdf/roboost_mecanum_robot.urdf.xacro )"
```

## Joint state publisher GUI

To debug the robot_state_publisher, the joint_state_publisher_gui can be used. It allows to manually set the joint states and see the resulting transformations. This will eventually be done by the Roboost Primary Motor Cortex, but for now it can be executed as a standalone node with the following command:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Rviz2

Rviz2 is a 3D visualization tool for ROS2. It can be used to visualize the robot model and the sensor data. To run it, execute the following command:

```bash
rviz2
```

Here you need to select the Fixed Frame to be "base_link" and add a RobotModel. You can also add a TF display to see the transformations.