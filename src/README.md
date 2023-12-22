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

### roboost

TODO: add description

#### launch files

##### camera.launch.py

This launch file will launch the camera node. This is needed to publish the camera stream and to make the camera available to other nodes.

```bash
ros2 launch roboost camera.launch.py
```

##### joy_control.launch.py

This launch file will launch the joy_control node. This is needed to control the robot with a joystick.

```bash
ros2 launch roboost joy_control.launch.py
```

###### mecanum_sim.launch.py

If you do not have a robot, you can launch the mecanum_sim.launch.py file to launch the robot in Gazebo. This will also launch the mecanum_tf_broadcast.

```bash
ros2 launch roboost mecanum_sim.launch.py world:=src/roboost/worlds/home.world
```

##### mecanum_tf_broadcast.launch.py

This launch file will launch the robot_state_publisher and the odom_to_base_node. This is needed to publish the tf tree for the robot and to make the urdf model available to other nodes.

```bash
ros2 launch tf_broadcast_package mecanum_tf_broadcast_launch.py
```

### [yolov8_ros](https://github.com/mgonzs13/yolov8_ros)

#### Test Usage

To launch the package, run the following commands:

For the camera stream:

```bash
ros2 launch roboost camera.launch.py
```

For the yolov8 node:

```bash
ros2 launch yolov8_bringup yolov8.launch.py device:=cpu input_image_topic:=/image_raw
```

For rqt image view:

```bash
ros2 run rqt_image_view rqt_image_view
```

### [slam-toolbox](https://github.com/SteveMacenski/slam_toolbox)

To start mapping, launch the slam toolbox using the provided config:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/roboost/config/mapper_params_online_async.yaml
```
