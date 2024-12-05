#! /usr/bin/env python3
import os
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from collections import defaultdict
from store_msgs.msg import PathXY  # 우리가 만든 메시지 타입입니다.

"""
PathXY :


builtin_interfaces/Time stamp
int16 x
int16 y
"""
# 파라미터
TIMER = 0.02
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

CLASS_COLORS = {
    0: (255, 0, 0),  # Class 0: Red
    1: (0, 255, 0),  # Class 1: Green
}


class OBJNode(Node):
    """
    USB카메라를 사용하여 객체를 인식하고 추적하는 노드
    """

    def __init__(self):
        super().__init__("obj_node")
        # 카메라 초기 설정
        self.cam_open()
        # 모델 초기 설정
        self.model_open()
        # 노드 초기 설정
        self.callback_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(TIMER, self.main_callback)

        self.publisher_TB = self.create_publisher(
            PathXY, "/path_TB", 10, callback_group=self.callback_group
        )
        self.publisher_RC = self.create_publisher(
            PathXY, "/path_RC", 10, callback_group=self.callback_group
        )
        self.publisher_state = self.create_publisher(
            Int8, "/state", 10, callback_group=self.callback_group
        )
        self.publisher_image = self.create_publisher(
            CompressedImage, "/compressdimage", 10, callback_group=self.callback_group
        )

        self.subscription = self.create_subscription(
            Int8, "/state", self.state_callback, 10, callback_group=self.callback_group
        )
        self.subscription
        # 클래스 변수 초기화
        self.state = 0

    def cam_open(self):
        """
        카메라 초기 설정 함수
        카메라를 열 수 없으면 인덱스를 바꿔서 다시 시도합니다.
        """
        idx = 4
        while True:
            self.cap = cv2.VideoCapture(idx)
            if not self.cap.isOpened():
                print(f"{idx} 카메라를 열 수 없습니다.")
                idx -= 2
            else:
                break
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    def model_open(self):
        """
        모델 초기 설정 함수
        """
        # 인식 모델
        self.model_predict = YOLO("/home/amtl/Desktop/rokey_e04/utile/detection.pt")
        # 추적 모델
        self.model_track = YOLO("/home/amtl/Desktop/rokey_e04/utile/track.pt")
        self.track_history = defaultdict(lambda: [])

    def top_image(self, img):
        """
        변환 행렬을 구하고 이미지를 탑뷰로 만드는 함수
        """
        matrix = cv2.getPerspectiveTransform(SRC, DST)
        top_view = cv2.warpPerspective(img, matrix, (WIDTH, HEIGHT))
        return top_view

    def state_callback(self, msg):
        """
        손님이 GUI를 통해 사용 여부 정보를 처리하는 콜백
        """
        self.state = msg.data

    def img_pub(self, img):
        """
        인식,추척 파악을 위한 이미지
        """
        _, buffer = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 10])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.publisher_image.publish(msg)

    def state_pub(self):
        """
        손님 인식 후 GUI에게 정보를 보내는 함수
        """
        msg = Int8()
        msg.data = 0
        self.publisher_state.publish(msg)

    def pathxy_pub(self, class_id, x, y):
        """
        객체의 좌표 토픽을 발행하는 함수
        """
        msg = PathXY()
        msg.stamp = self.get_clock().now().to_msg()
        msg.x = int(x)
        msg.y = int(y)
        if class_id == 0:
            self.publisher_RC.publish(msg)
        elif class_id == 1:
            self.publisher_TB.publish(msg)

    def detection(self, image: np.ndarray) -> np.ndarray:
        """
        ROI 영역에 RC(손님)이 있는지 감지하는 함수
        ROI 영역 -> 예측 + 토픽 발행 -> 시각 처리
        """
        ROI_image = image[int(HEIGHT / 3 * 1.5) : HEIGHT, 0 : int(WIDTH / 3 - 11)]
        ROI_height, ROI_width, _ = ROI_image.shape
        results = self.model_predict(ROI_image, save=False, imgsz=640, conf=0.9)
        detections = results[0].boxes
        color = (255, 255, 255)
        if detections is not None:
            for detection in detections:
                score = detection.conf
                class_id = int(detection.cls)
                if score[0] >= 0.9 and class_id == 0:
                    color = (255, 0, 0)
                    self.state_pub()
        yolo_view = results[0].plot()
        cv2.rectangle(
            yolo_view, (0, 0), (ROI_width - 1, ROI_height - 1), color, thickness=10
        )
        image[int(HEIGHT / 3 * 1.5) : HEIGHT, 0 : int(WIDTH / 3 - 11)] = yolo_view
        return image

    def track(self, image: np.ndarray) -> np.ndarray:
        """
        TOP view 이미지에 객체를 추적하고 이미지 좌표를 토픽으로 발행
        이미지 픽셀 좌표는 int형으로 int16으로 충분하게 표현 가능.
        """
        results = self.model_track.track(
            image, conf=0.25, iou=0.25, persist=True, device=0
        )
        if not results or not hasattr(results[0], "boxes") or results[0].boxes is None:
            print("No objects detected in the image.")
            return image
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        track_cls = results[0].boxes.cls.int().cpu().tolist()
        image = results[0].plot()
        for box, track_id, track_cl in zip(boxes, track_ids, track_cls):
            x, y, w, h = box
            self.pathxy_pub(track_cl, x, y)
            track = self.track_history[track_id]
            track.append((float(x), float(y)))
            #
            if len(track) > 25:
                track.pop(0)
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                image,
                [points],
                isClosed=False,
                color=CLASS_COLORS[track_cl],
                thickness=10,
            )
        return image

    def main_callback(self):
        """
        타이머 콜백
        일정 시간동안 노드의 메인 기능을 수행하는 함수
        """
        ret, frame = self.cap.read()
        top_view = self.top_image(frame)
        if self.state == 0:
            pub_image = self.detection(top_view)
        elif self.state == 1:
            pub_image = self.track(top_view)
        self.img_pub(pub_image)


def main(args=None):
    rclpy.init(args=args)
    node = OBJNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
