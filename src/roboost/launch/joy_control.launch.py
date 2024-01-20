import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    joy_params = os.path.join(
        get_package_share_directory("roboost"), "config", "joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[{"use_sim_time": use_sim_time}, joy_params],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("roboost"), "config", "twist_mux.yaml"
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[{"use_sim_time": use_sim_time}, twist_mux_params],
        remappings=[{"cmd_vel_out": "/cmd_vel"}],
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_node,
            twist_mux_node,
        ]
    )
