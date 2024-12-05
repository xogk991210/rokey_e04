# This program runs on Window due to WSL's limitation on integrating USB Camera
import numpy as np
import cv2
import os
from ultralytics import YOLO  # YOLOv8 모델

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


def warp_image(img):
    matrix = cv2.getPerspectiveTransform(SRC, DST)
    top_view = cv2.warpPerspective(img, matrix, (WIDTH, HEIGHT))
    return top_view


def capture_image():
    cap = cv2.VideoCapture(4)  # PC Camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    model = YOLO("/home/amtl/Desktop/rokey_e04/runs/detect/train/weights/best.pt")
    state = 0
    while True:
        ret, frame = cap.read()
        # top_view = warp_image(frame)
        # ww = top_view[int(HEIGHT / 3 * 1.5) : HEIGHT, 0 : int(WIDTH / 3 - 11)]
        # print(ww.shape)
        results = model.predict(ww, save=False, imgsz=640, conf=0.9)
        detections = results[0].boxes
        color = (255, 255, 255)
        if detections is not None:
            for detection in detections:
                score = detection.conf  # 신뢰도 점수
                class_id = int(detection.cls)  # 클래스 ID
                if score[0] >= 0.9 and class_id == 0:
                    color = (255, 0, 0)
                    state = 1
                else:
                    color = (0, 255, 0)
        yolo_view = results[0].plot()
        height, width, _ = yolo_view.shape
        cv2.rectangle(
            yolo_view,
            (0, 0),
            (width - 1, height - 1),
            color,
            thickness=10,
        )

        # print(results)
        # top_view[int(HEIGHT / 3 * 1.5) : HEIGHT, 0 : int(WIDTH / 3 - 11)] = yolo_view
        cv2.imshow("Webcam", ret, frame=cap.read())

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    capture_image()


if __name__ == "__main__":
    main()
