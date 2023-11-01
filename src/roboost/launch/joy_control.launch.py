import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory("roboost"), "config", "joystick.yaml"
    )

    print("joy_params: {}".format(joy_params))

    # Print the joy_params file to the console
    with open(joy_params, "r") as f:
        print(f.read())

    # close the file
    f.close()

    return LaunchDescription(
        [
            Node(package="joy", executable="joy_node"),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_node",
                parameters=[joy_params],
            ),
        ]
    )
