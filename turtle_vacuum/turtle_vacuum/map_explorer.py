#! usr/bin/env python3
"""
지도에서 추가적으로 탐사가 필요한 위치를 찾는 노드.
/map 토픽 
    0 : 자유 공간
    -1 : 미탐사 공간
    100 : 장애물 공간
방식은 2종류입니다.
    -단순한게 확인 방법
        1. 반복문으로 map의 자유 공간을 찾는다.
        2. 자유 공간의 주변 픽셀에서 -1이 있는지 확인한다.
        3. 조건에 맞으면 반환하는 배열에 추가한다.
    -이미지 연산 방법
        1. map을 하나의 이미지로 생각하고 자유 공간을 추출하여 팽창 시킨다.
        2. 팽창 자유 공간과 미탐사 공간의 두 이미지를 AND연산으로 경계를 찾는다.
        3. 장애물 공간 마스크를 약간 팽창시켜 제외시킨다.
이제 클러스터링하고 로봇과 가까운 곳의 좌표를 마킹한다.
"""
import cv2
import sys
import rclpy
import pathlib
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from utils import *


class MapExplorer(Node):
    def __init__(self):
        super().__init__("map_explorer")
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/bot4/map", self.map_callback, 10
        )
        self.marker_pub = self.create_publisher(Marker, "explorer_point", 10)
        self.map_info = None

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        map_explorer = empty_boundaries_with_mask(map_data)
        center = explorer_cluster(map_explorer)
        self.publish_center(center)

    def publish_center(self, center):
        """
        rviz에 시각화하기 편하게 마커표시
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.map_info.resolution * 3
        marker.scale.y = self.map_info.resolution * 3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.get_logger().info(f"센터 {len(center)}개")
        txt = ""
        for idx in center:
            y_idx, x_idx = idx
            x, y = grid_to_world(x_idx, y_idx, self.map_info)
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)
            txt = f"\t\tx : {x:.2f} y : {y:.2f}"
            self.get_logger().info(txt)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MapExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
