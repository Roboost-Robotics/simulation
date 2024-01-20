#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        # Publisher now publishes CompressedImage messages
        self.publisher_ = self.create_publisher(
            CompressedImage, "camera/image_compressed", qos_profile
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(1)

    def timer_callback(self):
        """
        Publishes the compressed image from the camera.
        :return: None
        """
        ret, frame = self.cap.read()
        if ret:
            # Resize the frame to a lower resolution, e.g., 640x480
            frame = cv2.resize(frame, (640, 480))
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Compress the frame to JPEG format
            ret, jpeg_frame = cv2.imencode(".jpg", frame)
            if ret:
                # Convert the compressed frame to a ROS 2 message
                msg = self.bridge.cv2_to_compressed_imgmsg(jpeg_frame, dst_format="jpg")

                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
