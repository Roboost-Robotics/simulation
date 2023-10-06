import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs

class OdomToBaseLinkTfNode(Node):

    def __init__(self):
        super().__init__('odom_to_base_link_tf_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg() # TODO: Change timestamp to timestamp of message
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'

        transform_stamped.transform.translation.x = msg.pose.pose.position.x
        transform_stamped.transform.translation.y = msg.pose.pose.position.y
        transform_stamped.transform.translation.z = msg.pose.pose.position.z

        transform_stamped.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    odom_to_base_link_tf_node = OdomToBaseLinkTfNode()
    rclpy.spin(odom_to_base_link_tf_node)
    odom_to_base_link_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
