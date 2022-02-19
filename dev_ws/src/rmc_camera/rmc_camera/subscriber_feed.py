import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2 as cv


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("subscriber_feed")
        self.subscription = self.create_subscription(
            Image, "camera_feed", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info("getting data")

        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg)

        # Display image
        # cv.imshow("camera", current_frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
