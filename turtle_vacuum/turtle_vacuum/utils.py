#! usr/bin/env python3
import numpy
import cv2
from sklearn.cluster import DBSCAN

DIRECTIONS = [
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
]


def empty_with_neighbors(map: numpy.ndarray) -> list:
    """
    입력 : map(배열)
    출력 : map_explorer(리스트)
    1. 반복문에서 주변 셀을 탐색하기 위해 패딩
    2. 0(자유 공간) 찾기
    3. 자유 공간을 이용해서 반복문으로 셀의 주변 셀 값을 비교하여 -1(미탐사 공간)이면 저장
    """
    map_explorer = []
    padded_map = numpy.pad(map, pad_width=1, mode="constant", constant_values=100)
    free_space = numpy.argwhere(padded_map == 0)

    for x, y in free_space:
        for dy, dx in DIRECTIONS:
            ny, nx = x + dy, y + dx
            if padded_map[ny, nx] == -1:
                map_explorer.append((y, x))
                break
    return map_explorer


def empty_boundaries_with_mask(map: numpy.ndarray) -> numpy.ndarray:
    """
    입력 : map(배열)
    출력 : map_explorer(리스트)
    장애물(100)은 0, 자유 공간(0)은 255, 미탐색 공간은(-1)은 127
    1. map을 이미지 연산을 위해 map이미지 만들기
    2. 장애물 마스크 만들고 확장
    3. map이미지에 확장 장애물 적용
        최종적으로 자유 공간과 미탐색 공간의 경계를 알아야함. 단 두 공간 사이에 벽 같은 장애물이 있으면 경계에서 제외.
        자유공간 확장 후 연산의 결과로 얇은 장애물 공간이 사라질 수 있음
        따라서 확장 장애물 마스크를 map이미지에 적용시켜 벽과 가까운 자유, 미탐사 영역에 덮어 장애물 공간 주변에 두 공간의 겹침을 최소화하여 결과값을 안정화.
    4.확장 자유 공간, 미탐사 공간 AND 연산 후 장애물 공간을 제외
    """
    kernel = numpy.ones((5, 5), numpy.uint8)
    image = numpy.zeros(map.shape, dtype=numpy.uint8)
    image[map == -1] = 127
    image[map == 0] = 255
    image[map == 100] = 0

    wall_mask = numpy.zeros_like(image)
    wall_mask[image == 0] = 255
    dilated_wall = cv2.dilate(wall_mask, kernel, iterations=1)

    image[dilated_wall == 255] = 0

    free_space = numpy.zeros_like(image)
    free_space[image == 255] = 255
    dilated_free_space = cv2.dilate(free_space, kernel, iterations=1)

    unknown_space = numpy.zeros_like(image)
    unknown_space[image == 127] = 255

    map_explorer = cv2.bitwise_and(dilated_free_space, unknown_space)
    map_explorer = cv2.subtract(map_explorer, dilated_wall)
    map_explorer = numpy.column_stack(numpy.where(map_explorer == 255))
    return map_explorer


def explorer_cluster(map_explorer: numpy.ndarray):
    """
    입력 : map_explorer(배열)
    출력 : center(리스트)
    Scikit-learn의 DBSCAN 알고리즘을 사용하여 클러스터링
    걍 라이브러리 가져와서 모델에 데이터 넣고 센터 반환
    """
    center = []
    if len(map_explorer) > 0:
        clustering = DBSCAN(eps=5, min_samples=5).fit(map_explorer)
        labels = clustering.labels_

        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            class_member_mask = labels == label
            cluster_points = map_explorer[class_member_mask]
            centroid = numpy.mean(cluster_points, axis=0)
            center.append(centroid)

    return center


def grid_to_world(x_idx, y_idx, map_info):
    x = (
        x_idx * map_info.resolution
        + map_info.origin.position.x
        + map_info.resolution / 2
    )
    y = (
        y_idx * map_info.resolution
        + map_info.origin.position.y
        + map_info.resolution / 2
    )
    return x, y
