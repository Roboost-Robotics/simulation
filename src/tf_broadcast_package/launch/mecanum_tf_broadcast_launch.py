import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    urdf_file_name = "roboost_mecanum_robot.urdf.xacro"

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory("tf_broadcast_package"), urdf_file_name
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": Command(["xacro", " ", urdf]),
                        "ignore_timestamp": "true", # TODO: Is this necessary?
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui", # TODO: Is this necessary?
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time
                    }
                ],
            ),
            Node(
                package="tf_broadcast_package",
                executable="odom_to_base_node",
                name="odom_to_base_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
