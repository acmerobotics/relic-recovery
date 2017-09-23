import cv2
import numpy as np
import os

RED_LOWER_HSV, RED_UPPER_HSV = (0, 80, 0), (7, 255, 255)
RED2_LOWER_HSV, RED2_UPPER_HSV = (170, 80, 0), (180, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (110, 80, 0), (130, 255, 255)

OPEN_KERNEL_SIZE, CLOSE_KERNEL_SIZE = 7, 15

IMAGE_DIR = 'cryptobox/'
MIN_DIMENSION = 480


def format_num(num):
    if num >= 10**6:
        return '{:.1f}M'.format(num / 10**6)
    elif num >= 10**3:
        return '{:.1f}K'.format(num / 10**3)
    else:
        return str(num)


os.makedirs('cryptobox2/', exist_ok=True)
for i, filename in enumerate(os.listdir(IMAGE_DIR)):
    image = cv2.imread(IMAGE_DIR + filename)
    height, width = image.shape[:2]
    if height < width:
        new_height = MIN_DIMENSION
        new_width = int(width / height * new_height)
    else:
        new_width = MIN_DIMENSION
        new_height = int(height / width * new_width)
    image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, RED_LOWER_HSV, RED_UPPER_HSV)
    red2 = cv2.inRange(hsv, RED2_LOWER_HSV, RED2_UPPER_HSV)
    blue = cv2.inRange(hsv, BLUE_LOWER_HSV, BLUE_UPPER_HSV)

    cv2.bitwise_or(red, red2, red)

    open_kernel = np.ones((OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE), np.uint8)
    close_kernel = np.ones((CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE), np.uint8)
    red_morph = cv2.morphologyEx(red, cv2.MORPH_OPEN, open_kernel)
    red_morph = cv2.morphologyEx(red_morph, cv2.MORPH_CLOSE, close_kernel)
    blue_morph = cv2.morphologyEx(blue, cv2.MORPH_OPEN, open_kernel)
    blue_morph = cv2.morphologyEx(blue_morph, cv2.MORPH_CLOSE, close_kernel)

    red_contours = cv2.findContours(red_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
    blue_contours = cv2.findContours(blue_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    output = image.copy()

    red_areas = []
    blue_areas = []

    for contour in red_contours:
        area = int(cv2.contourArea(contour))
        rect = cv2.boundingRect(contour)
        if (rect[2] / rect[3]) > 0.5 or area < 750:
            continue
        red_areas.append(area)
        cv2.drawContours(output, [contour], 0, (0, 0, 255), 3)
        vx, vy, x, y = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        cv2.line(output, (x, 0), (x, new_height), (0, 255, 255), 2)

    for contour in blue_contours:
        area = int(cv2.contourArea(contour))
        rect = cv2.boundingRect(contour)
        if (rect[2] / rect[3]) > 0.5 or area < 750:
            continue
        blue_areas.append(area)
        cv2.drawContours(output, [contour], 0, (255, 0, 0), 3)
        vx, vy, x, y = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        cv2.line(output, (x, 0), (x, new_height), (255, 255, 0), 2)

    red_areas.sort(reverse=True)
    blue_areas.sort(reverse=True)

    for j, area in enumerate(red_areas):
        cv2.putText(output, format_num(area), (0, 30 * (j + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 8)
        cv2.putText(output, format_num(area), (0, 30 * (j + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    for j, area in enumerate(blue_areas):
        cv2.putText(output, format_num(area), (0, 30 * (j + 1 + len(red_areas))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 8)
        cv2.putText(output, format_num(area), (0, 30 * (j + 1 + len(red_areas))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    base_name = filename.split('.')[0]
    cv2.imwrite('cryptobox2/{}_0.jpg'.format(base_name), image)
    cv2.imwrite('cryptobox2/{}_1_red_thresh.jpg'.format(base_name), red)
    cv2.imwrite('cryptobox2/{}_3_blue_thresh.jpg'.format(base_name), blue)
    cv2.imwrite('cryptobox2/{}_2_red_morph.jpg'.format(base_name), red_morph)
    cv2.imwrite('cryptobox2/{}_4_blue_morph.jpg'.format(base_name), blue_morph)
    cv2.imwrite('cryptobox2/{}_5_output.jpg'.format(base_name), output)