import numpy as np
import cv2
from math import sqrt, log
from os import listdir
from util import smart_hsv_range
from matplotlib import pyplot as plt
from collections import defaultdict

RED_LOWER_HSV, RED_UPPER_HSV = (170, 80, 0), (7, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (100, 80, 0), (124, 255, 255)

INPUT_DIR_NAME = "cryptobox-with-tape/images/"
OUTPUT_DIR_NAME = "cryptobox-with-tape/output/"


# see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
def distance_between_point_and_line(point1, point2, point3):
    euclidean_distance = sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    return abs((point2[1] - point1[1]) * point3[0] - (point2[0] - point1[0]) * point3[1]
               + point2[0] * point1[1] - point2[1] * point1[0]) / (euclidean_distance + 0.0001)


def process_image(image, lower_hsv=BLUE_LOWER_HSV, upper_hsv=BLUE_UPPER_HSV):
    height, width, _ = image.shape
    resize = cv2.resize(image, (640, int(height / width * 640)))

    hsv = cv2.cvtColor(resize, cv2.COLOR_BGR2HSV)
    hsv_mask = smart_hsv_range(hsv, lower_hsv, upper_hsv)
    morph_hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, np.ones((9, 9), dtype=np.uint8))
    morph_hsv_mask = cv2.morphologyEx(morph_hsv_mask, cv2.MORPH_CLOSE, np.ones((41, 5), dtype=np.uint8))

    gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_and(gray, morph_hsv_mask)
    _, mask = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
    morph_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))
    morph_mask = cv2.morphologyEx(morph_mask, cv2.MORPH_CLOSE, np.ones((9, 9), dtype=np.uint8))

    _, contours, _ = cv2.findContours(morph_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    output = resize.copy()
    points = []
    for contour in contours:
        M = cv2.moments(contour)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        points.append((cx, cy))

    outputs = {}
    outputs['0_resize'] = resize
    outputs['1_hsv_mask'] = hsv_mask
    outputs['2_morph_hsv_mask'] = morph_hsv_mask
    outputs['3_gray'] = gray
    outputs['4_mask'] = mask
    outputs['5_morph_mask'] = morph_mask
    outputs['6_output'] = output

    return points, outputs


# Check if a point is inside a rectangle
def rect_contains(rect, point):
    if point[0] < rect[0]:
        return False
    elif point[1] < rect[1]:
        return False
    elif point[0] > rect[2]:
        return False
    elif point[1] > rect[3]:
        return False
    return True


def dist(pt1, pt2):
    return sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)


def get_delaunay_triangulation(image, points):
    size = image.shape
    r = (0, 0, size[1], size[0])
    subdiv = cv2.Subdiv2D(r)
    for point in points:
        subdiv.insert(point)
    return subdiv.getTriangleList()


def draw_delaunay_triangulation(image, triangulation):
    size = image.shape
    r = (0, 0, size[1], size[0])
    for t in triangulation:
        pt1 = (t[0], t[1])
        pt2 = (t[2], t[3])
        pt3 = (t[4], t[5])
        if rect_contains(r, pt1) and rect_contains(r, pt2) and rect_contains(r, pt3):
            cv2.line(image, pt1, pt2, (255, 0, 255), 5)
            cv2.line(image, pt2, pt3, (255, 0, 255), 5)
            cv2.line(image, pt3, pt1, (255, 0, 255), 5)


def main():
    for filename in listdir(INPUT_DIR_NAME):
        image = cv2.imread(INPUT_DIR_NAME + filename)
        if 'red' in filename:
            points, outputs = process_image(image, RED_LOWER_HSV, RED_UPPER_HSV)
        else:
            points, outputs = process_image(image, BLUE_LOWER_HSV, BLUE_UPPER_HSV)

        for name, image in outputs.items():
            if 'output' in name:
                triangulation = get_delaunay_triangulation(image, points)
                draw_delaunay_triangulation(image, triangulation)
            output_filename = '{}{}_{}.jpg'.format(OUTPUT_DIR_NAME, filename.split('.')[0], name)
            cv2.imwrite(output_filename, image)
        print('processed {}'.format(filename))


if __name__ == '__main__':
    main()
