#!/usr/bin/env python3
import os
import random
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory


class AudioPlayer(Node):
    def __init__(self):
        super().__init__("audio_player_node")
        print("Initializing audio player node")

        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        package_share_directory = get_package_share_directory("roboost")
        self.audio_directory = os.path.join(package_share_directory, "audio")
        self.last_state = False

    def joy_callback(self, msg):
        """
        Callback function for the /joy topic.
        :param msg: Joy message
        :return: None
        """
        current_state = (
            msg.buttons[0] == 1
        )  # Check if 'A' button (first button) is pressed

        if current_state and not self.last_state:
            self.play_random_sound()

        self.last_state = current_state

    def play_random_sound(self):
        """
        Play a random sound from the audio directory.
        :return: None
        """
        if os.path.exists(self.audio_directory) and os.path.isdir(self.audio_directory):
            files = [f for f in os.listdir(self.audio_directory) if f.endswith(".ogg")]
            if files:
                random_file = random.choice(files)
                file_path = os.path.join(self.audio_directory, random_file)
                subprocess.run(
                    [
                        "cvlc",
                        "--play-and-exit",
                        "--gain",
                        "10",
                        "--audio-filter=volnorm",
                        file_path,
                    ]
                )
            else:
                self.get_logger().warn("No .ogg files found in the audio directory")
        else:
            self.get_logger().warn(
                "Audio directory does not exist or is not a directory"
            )


def main(args=None):
    rclpy.init(args=args)
    audio_player_node = AudioPlayer()
    print("Spinning audio player node. Press 'A' on the controller to play a sound")
    rclpy.spin(audio_player_node)
    audio_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
