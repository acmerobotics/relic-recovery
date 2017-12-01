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


def find_grid(points):
    point_scores = defaultdict(lambda : 0)
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            point1, point2 = points[i], points[j]
            slope = (point2[1] - point1[1]) / (point2[0] - point1[0] + 0.0001)
            inverse_slope = 1 / (slope + 0.0001)
            score = min(abs(slope), abs(inverse_slope))
            point_scores[point1] += score
            point_scores[point2] += score
    return point_scores


def process_image(image, lower_hsv=BLUE_LOWER_HSV, upper_hsv=BLUE_UPPER_HSV):
    height, width, _ = image.shape
    resize = cv2.resize(image, (640, int(height / width * 640)))
    # resize = cv2.GaussianBlur(resize, (5, 5), 0)

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

    slopes = []
    inverse_slopes = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            point1, point2 = points[i], points[j]
            slope = (point2[1] - point1[1]) / (point2[0] - point1[0] + 0.0001)
            slopes.append(max(min(slope, 15), -15))
            inverse_slopes.append(max(min(1.0 / (slope + 0.0001), 5), -5))

    # for point in points:
    #     cv2.circle(output, point, 10, (255, 0, 255), cv2.FILLED)

    outputs = {}
    outputs['0_resize'] = resize
    outputs['1_hsv_mask'] = hsv_mask
    outputs['2_morph_hsv_mask'] = morph_hsv_mask
    outputs['3_gray'] = gray
    outputs['4_mask'] = mask
    outputs['5_morph_mask'] = morph_mask
    outputs['6_output'] = output
    outputs['7_slopes'] = slopes
    outputs['8_inverse_slopes'] = inverse_slopes

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


def main():
    for filename in listdir(INPUT_DIR_NAME):
        image = cv2.imread(INPUT_DIR_NAME + filename)
        if 'red' in filename:
            points, outputs = process_image(image, RED_LOWER_HSV, RED_UPPER_HSV)
        else:
            points, outputs = process_image(image, BLUE_LOWER_HSV, BLUE_UPPER_HSV)
        # cv2.imwrite(OUTPUT_DIR_NAME + filename, output)
        for key, value in outputs.items():
            if 'output' in key:
                # point_scores = find_grid(points)
                # max_score = max(point_scores.values())
                # multiplier = 15 / max_score
                # for point, score in point_scores.items():
                #     cv2.circle(value, point, int(score * multiplier), (255, 255, 0), cv2.FILLED)
                size = value.shape
                r = (0, 0, size[1], size[0])
                subdiv = cv2.Subdiv2D(r)
                for point in points:
                    subdiv.insert(point)
                triangleList = subdiv.getTriangleList()

                for t in triangleList:
                    pt1 = (t[0], t[1])
                    pt2 = (t[2], t[3])
                    pt3 = (t[4], t[5])

                    if rect_contains(r, pt1) and rect_contains(r, pt2) and rect_contains(r, pt3):
                        slope1 = (abs(pt1[1] - pt2[1]) + 0.0001) / (abs(pt1[0] - pt2[0]) + 0.0001)
                        slope2 = (abs(pt2[1] - pt3[1]) + 0.0001) / (abs(pt2[0] - pt3[0]) + 0.0001)
                        slope3 = (abs(pt3[1] - pt1[1]) + 0.0001) / (abs(pt3[0] - pt1[0]) + 0.0001)
                        if abs(log(slope1)) > log(3):
                            cv2.line(value, pt1, pt2, (255, 0, 255), 5)
                        if abs(log(slope2)) > log(3):
                            cv2.line(value, pt2, pt3, (255, 0, 255), 5)
                        if abs(log(slope3)) > log(3):
                            cv2.line(value, pt3, pt1, (255, 0, 255), 5)

                for point in points:
                    cv2.circle(value, point, 10, (255, 255, 0), cv2.FILLED)
            else:
                continue

            output_filename = '{}{}_{}.jpg'.format(OUTPUT_DIR_NAME, filename.split('.')[0], key)
            if type(value) is list:
                plt.figure()
                plt.hist(value, 60, [-15, 15])
                plt.title(key)
                plt.savefig(output_filename)
            else:
                cv2.imwrite(output_filename, value)
        print('processed {}'.format(filename))


if __name__ == '__main__':
    main()
