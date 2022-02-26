import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2

from cv_bridge import CvBridge

# from . import registry

from ros2_numpy.ros2_numpy import ros2_numpy as rn


class ObstructionPublisher(Node):
    def __init__(self):
        super().__init__("publisher_obstruction")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "camera_feed", self.listener_callback, 10
        )

        self.publisher_ = self.create_publisher(
            Image, "publisher_obstruction", 10
        )

    def listener_callback(self, msg):
        """
        timer call back function for periodically publishing camera feed data
        """

        EdgeArray = []
        img = self.bridge.imgmsg_to_cv2(msg)

        imgGray = cv2.cvtColor(
            img, cv2.COLOR_BGR2GRAY
        )  # convert img to grayscale and store result in imgGray

        imgGray = cv2.bilateralFilter(
            img, 9, 30, 30
        )  # blur the image slightly to remove noise
        imgEdge = cv2.Canny(imgGray, 0, 300)  # edge detection

        imagewidth = imgEdge.shape[1] - 1
        imageheight = imgEdge.shape[0] - 1
        StepSize = 20

        for j in range(
            0, imagewidth, StepSize
        ):  # for the width of image array
            for i in range(
                imageheight - 5, 0, -1
            ):  # step through every pixel in height of array from bottom to top
                # Ignore first couple of pixels as may trigger due to undistort
                if (
                    imgEdge.item(i, j) == 255
                ):  # check to see if the pixel is white which indicates an edge has been found
                    EdgeArray.append(
                        (j, i)
                    )  # if it is, add x,y coordinates to ObstacleArray
                    break  # if white pixel is found, skip rest of pixels in column
            else:  # no white pixel found
                EdgeArray.append(
                    (j, 0)
                )  # if nothing found, assume no obstacle. Set pixel position way off the screen to indicate
                # no obstacle detected

        for x in range(
            len(EdgeArray) - 1
        ):  # draw lines between points in ObstacleArray
            cv2.line(img, EdgeArray[x], EdgeArray[x + 1], (255, 0, 0), 1)
        for x in range(
            len(EdgeArray)
        ):  # draw lines from bottom of the screen to points in ObstacleArray
            cv2.line(
                img, (x * StepSize, imageheight), EdgeArray[x], (255, 0, 0), 1
            )

        self.publisher_.publish(rn.msgify(Image, EdgeArray))
        self.get_logger().info("sent obstruction data")


def main(args=None):
    rclpy.init(args=args)
    publisher = ObstructionPublisher()
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
