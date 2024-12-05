import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CompressedVideoSubscriber(Node):
    def __init__(self):
        super().__init__("compressed_video_subscriber")

        self.subscription = self.create_subscription(
            CompressedImage, "compressdimage", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        # 압축된 이미지를 디코드
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 화면에 표시
        cv2.imshow("Compressed Video Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quitting...")
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CompressedVideoSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
