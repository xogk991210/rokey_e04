import rclpy
import numpy as np
import cv2
import os
from rclpy.node import Node
from store_msgs.msg import PathXY  # 우리가 만든 메시지 타입입니다.

"""
PathXY :


builtin_interfaces/Time stamp
int16 x
int16 y
"""
TIMER = 0.05
WIDTH = 1280
HEIGHT = 720


class CMDNode(Node):
    def __init__(self):
        super().__init__("cmd_node")
        self.timer = self.create_timer(TIMER, self.main_callback)
        self.subscription = self.create_subscription(
            PathXY, "path_TB", self.TB_callback, 10
        )
        self.subscription = self.create_subscription(
            PathXY, "path_RC", self.RC_callback, 10
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
