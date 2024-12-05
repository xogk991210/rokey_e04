import rclpy
import numpy as np
import cv2
import os
from rclpy.node import Node
from nav_msgs.msg import Path
from ultralytics import YOLO  # YOLOv8 모델

"""
Path Message

std_msgs/Header header
geometry_msgs/PoseStamped[] poses

"""
TIMER = 0.05
WIDTH = 1280
HEIGHT = 720
SRC = np.array(
    [
        [293, 219],
        [1087, 211],
        [108, 640],
        [1231, 634],
    ],
    dtype=np.float32,
)
DST = np.array(
    [
        [0, 0],
        [WIDTH, 0],
        [0, HEIGHT],
        [WIDTH, HEIGHT],
    ],
    dtype=np.float32,
)


class CMDNode(Node):
    def __init__(self):
        super().__init__("cmd_node")
        self.timer = self.create_timer(TIMER, self.main_callback)
        self.subscription = self.create_subscription(
            Path, "path_TB", self.TB_callback, 10
        )
        self.subscription = self.create_subscription(
            Path, "path_RC", self.RC_callback, 10
        )
        self.subscription

    def main_callback(self):
        pass

    def TB_callback(self):
        pass

    def RC_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = CMDNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
