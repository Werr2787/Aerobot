import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class QRTest(Node):

    def __init__(self):

        super().__init__("qr_test")

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.sub = self.create_subscription(
            Image,
            "/uav1/camera_down",
            self.callback,
            10
        )

    def callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg,
            "bgr8"
        )

        text, points, _ = self.detector.detectAndDecode(
            image
        )

        if text:

            self.get_logger().info(
                f"QR FOUND: {text}"
            )


def main():

    rclpy.init()

    node = QRTest()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()