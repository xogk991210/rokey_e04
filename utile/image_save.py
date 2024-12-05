# This program runs on Window due to WSL's limitation on integrating USB Camera
import numpy as np

import cv2
import os

# import time

save_directory = (
    "img_capture"  # save direectory is set to be under the current directory
)

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

    os.makedirs(save_directory, exist_ok=True)

    # del_img = input("Do you want to delete the previous images: (y/n) ")
    # if del_img == 'y':
    #     file_name = f'{save_directory}\*.*'
    #     os.remove(file_name)
    #     print(f"{file_name} has been deleted.")

    file_prefix = input("Enter a file prefix to use : ")
    file_prefix = f"{file_prefix}_"
    print(file_prefix)

    image_count = 0
    cap = cv2.VideoCapture(4)  # PC Camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # cap = cv2.VideoCapture(1)   #USB Camera

    while True:
        ret, frame = cap.read()
        top_view = warp_image(frame)
        # ww = top_view[int(HEIGHT / 3 * 1.5) : HEIGHT, 0 : int(WIDTH / 3 - 11)]
        # cv2.imshow("Webcam", ww)
        # print(ww.shape)
        cv2.imshow("Webcam", top_view)

        key = cv2.waitKey(1)
        if key == ord("c"):

            # change the filename when multiple people are capturing images
            # ex: obj1_img_{image_count}.jpg
            # then all images and txt files generated can be combined for execution of step 3
            file_name = f"{save_directory}/{file_prefix}img_{image_count}.jpg"

            cv2.imwrite(file_name, top_view)
            print(f"Image saved. name:{file_name}")
            image_count += 1

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    capture_image()


if __name__ == "__main__":
    main()
