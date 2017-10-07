import cv2
import numpy as np
import os
from util import smart_hsv_range, resize_min_dim
from math import tan, pi

BLUE, RED, BROWN, GRAY = tuple(range(4))
RED_LOWER_HSV, RED_UPPER_HSV = (174, 80, 0), (4, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (112, 80, 0), (124, 255, 255)
# BROWN_LOWER_HSV, BROWN_UPPER_HSV = (1, 29, 55), (21, 77, 96)
# GRAY_LOWER_HSV, GRAY_UPPER_HSV = (60, 2, 120), (120, 27, 198)
BROWN_LOWER_HSV, BROWN_UPPER_HSV = (0, 31, 27), (19, 94, 104)
# BROWN_LOWER_HSV, BROWN_UPPER_HSV = (0, 20, 60), (178, 96, 130)
GRAY_LOWER_HSV, GRAY_UPPER_HSV = (66, 3, 121), (126, 69, 210)

OPEN_KERNEL_SIZE, CLOSE_KERNEL_SIZE = 5, 5

MIN_DIMENSION = 360

ACTUAL_COL_WIDTH = 7.5  # in
FOV_DEGREES = 58.5
FOCAL_LENGTH_IN = 3.2 / 25.4  # in

INPUT_DIR = 'cryptobox/images/'
OUTPUT_DIR = 'cryptobox/output/'

OPEN_KERNEL = np.ones((OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE), np.uint8)
CLOSE_KERNEL = np.ones((CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE), np.uint8)

os.makedirs(OUTPUT_DIR, exist_ok=True)
for i, filename in enumerate(os.listdir(INPUT_DIR)):
    base_name = filename.split('.')[0]

    image = cv2.imread(INPUT_DIR + filename)

    output = image.copy()

    # image = resize_min_dim(image, MIN_DIMENSION)
    image = cv2.pyrDown(image)
    image = cv2.pyrDown(image)

    image_height, image_width = image.shape[:2]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = smart_hsv_range(hsv, RED_LOWER_HSV, RED_UPPER_HSV)
    blue = smart_hsv_range(hsv, BLUE_LOWER_HSV, BLUE_UPPER_HSV)
    brown = smart_hsv_range(hsv, BROWN_LOWER_HSV, BROWN_UPPER_HSV)
    gray = smart_hsv_range(hsv, GRAY_LOWER_HSV, GRAY_UPPER_HSV)

    red_count = cv2.countNonZero(red)
    blue_count = cv2.countNonZero(blue)
    total = red_count + blue_count

    print("{}: \t{:.2f} R / {:.2f} B".format(filename, red_count / total, blue_count / total))

    if blue_count > red_count:
        crypto_image = blue
        output_color = (255, 255, 0)
    else:
        crypto_image = red
        output_color = (0, 255, 255)

    # find cryptobox
    crypto_morph = cv2.morphologyEx(crypto_image, cv2.MORPH_OPEN, OPEN_KERNEL)
    crypto_morph = cv2.morphologyEx(crypto_morph, cv2.MORPH_CLOSE, CLOSE_KERNEL)

    crypto_contours = cv2.findContours(crypto_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    cols = []
    for contour in crypto_contours:
        area = int(cv2.contourArea(contour))
        rect = cv2.boundingRect(contour)
        if (rect[2] / rect[3]) > 0.5 or area < 250:
            continue
        M = cv2.moments(contour)
        cx = int(M['m10'] / M['m00'])
        cols.append(cx)
        if output is not None:
            # cv2.drawContours(output, [contour], 0, output_color, 2)
            cv2.line(output, (4 * cx, 4 * rect[1]), (4 * cx, 4 * (rect[1] + rect[3])), output_color, 10)

    cols.sort()
    if len(cols) > 1:
        col_width = (max(cols) - min(cols)) / (len(cols) - 1)
        focal_length_px = (image_width * 0.5) / tan(FOV_DEGREES * 0.5 * pi / 180)
        distance = (ACTUAL_COL_WIDTH * focal_length_px) / col_width
        center = sum(cols) / len(cols)
        dx = ((0.5 * image_width - center) * distance) / focal_length_px

        cv2.putText(output, "{:.2f} in, {:.2f} in".format(distance, dx), (5, 50), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 4)
        # cv2.putText(output, str([int(i) for i in cols]), (5, 60), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)

    # find glyphs

    full_glyphs, potential_glyphs, partial_glyphs = [], [], []
    glyph_width_sum, glyph_width_count = 0, 0

    for glyph_image, color in ((brown, BROWN), (gray, GRAY)):
        glyph_morph = cv2.morphologyEx(glyph_image, cv2.MORPH_OPEN, OPEN_KERNEL)
        glyph_morph = cv2.morphologyEx(glyph_morph, cv2.MORPH_CLOSE, CLOSE_KERNEL)

        # cv2.imwrite('{}/{}_{}_morph.jpg'.format(OUTPUT_DIR, base_name, 'brown' if color == BROWN else 'gray'), glyph_morph)

        glyph_contours = cv2.findContours(glyph_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

        for contour in glyph_contours:
            area = cv2.contourArea(contour)
            rect = cv2.boundingRect(contour)
            rect_area = rect[2] * rect[3]
            aspect_ratio = rect[3] / rect[2]
            aspect_ratio_int = round(aspect_ratio)
            if area / rect_area > 0.85:
                if abs(aspect_ratio_int - aspect_ratio) < 0.25:
                    glyph_width, glyph_height = rect[2], rect[3] / aspect_ratio_int
                    glyph_width_sum += glyph_width
                    glyph_width_count += 1
                    for j in range(aspect_ratio_int):
                        x, y = rect[0], rect[1] + j * glyph_height
                        full_glyphs.append((x, y, glyph_width, glyph_height))
                else:
                    potential_glyphs.append(rect)

    if glyph_width_count > 0:
        mean_glyph_size = glyph_width_sum / glyph_width_count

        for rect in potential_glyphs:
            if rect[0] != 0 and rect[1] != 0 and (rect[0] + rect[2]) != image_width and (rect[1] + rect[3]) != image_height:
                continue

            width_ratio = rect[2] / mean_glyph_size
            width_ratio_int = round(width_ratio)
            height_ratio = rect[3] / mean_glyph_size
            height_ratio_int = round(height_ratio)

            if abs(width_ratio - width_ratio_int) < 0.2 or abs(height_ratio - height_ratio_int) < 0.2:
                partial_glyphs.append(rect)
                cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                              (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 127, 255), 10)
            else:
                cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                              (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 0, 0), 10)

    for rect in full_glyphs:
        cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                      (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 255, 0), 10)

    # cv2.imwrite('{}/{}_brown.jpg'.format(OUTPUT_DIR, base_name), brown)
    # cv2.imwrite('{}/{}_gray.jpg'.format(OUTPUT_DIR, base_name), gray)
    cv2.imwrite('{}/{}.jpg'.format(OUTPUT_DIR, base_name), output)