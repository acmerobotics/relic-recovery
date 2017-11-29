import numpy as np
import cv2
from math import sqrt
from os import listdir
from util import smart_hsv_range

RED_LOWER_HSV, RED_UPPER_HSV = (170, 80, 0), (7, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (100, 80, 0), (124, 255, 255)

INPUT_DIR_NAME = "R:/Downloads/cryptoboxes/"
OUTPUT_DIR_NAME = "R:/Downloads/output/"


# see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
def distance_between_point_and_line(point1, point2, point3):
    euclidean_distance = sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    return abs((point2[1] - point1[1]) * point3[0] - (point2[0] - point1[0]) * point3[1]
               + point2[0] * point1[1] - point2[1] * point1[0]) / (euclidean_distance + 0.0001)


def process_image(image, lower_hsv=BLUE_LOWER_HSV, upper_hsv=BLUE_UPPER_HSV, debug=False):
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

    # points_on_lines = []
    # for i in range(len(points)):
    #     for j in range(i + 1, len(points)):
    #         point1, point2 = points[i], points[j]
    #         min_dist, min_point = 1000, None
    #         for k in range(j + 1, len(points)):
    #             point3 = points[k]
    #             dist = distance_between_point_and_line(point1, point2, point3)
    #             if dist < min_dist:
    #                 min_dist = dist
    #                 min_point = point3
    #         if min_point is None:  # TODO: fix
    #             continue
    #         if min_dist < 5:
    #             slope = (point2[1] - point1[1]) / (point2[0] - point1[0] + 0.0001)
    #             intercept = point1[1] - slope * point1[0]
    #             cv2.line(output, (0, int(intercept)), (width, int(slope * width + intercept)), (0, 255, 255), 3)
    #             if abs(slope) < 0.1:
    #                 line_y = int((point1[1] + point2[1]) / 2)
    #                 cv2.line(output, (0, line_y), (width, line_y), (0, 255, 255), 3)
    #             if abs(slope) > 5:
    #                 line_x = int((point1[0] + point2[0]) / 2)
    #                 cv2.line(output, (line_x, 0), (line_x, height), (255, 255, 0), 3)
    #             if abs(slope) < 0.1 or abs(slope) > 5:
    #                 if point1 not in points_on_lines:
    #                     points_on_lines.append(point1)
    #                 if point2 not in points_on_lines:
    #                     points_on_lines.append(point2)

    for point in points:
        cv2.circle(output, point, 10, (255, 0, 255), cv2.FILLED)

    if debug:
        outputs = {}
        outputs['0_resize'] = resize
        outputs['1_hsv_mask'] = hsv_mask
        outputs['2_morph_hsv_mask'] = morph_hsv_mask
        outputs['3_gray'] = gray
        outputs['4_mask'] = mask
        outputs['5_morph_mask'] = morph_mask
        outputs['6_output'] = output
        return outputs

    return output


def main():
    for filename in listdir(INPUT_DIR_NAME):
        image = cv2.imread(INPUT_DIR_NAME + filename)
        if 'red' in filename:
            outputs = process_image(image, RED_LOWER_HSV, RED_UPPER_HSV, True)
        else:
            outputs = process_image(image, BLUE_LOWER_HSV, BLUE_UPPER_HSV, True)
        # cv2.imwrite(OUTPUT_DIR_NAME + filename, output)
        for key, value in outputs.items():
            cv2.imwrite('{}{}_{}.jpg'.format(OUTPUT_DIR_NAME, filename.split('.')[0], key), value)
        print('processed {}'.format(filename))


if __name__ == '__main__':
    main()
