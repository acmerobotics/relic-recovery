import cv2
import numpy as np
import os
from util import smart_hsv_range
from math import tan, pi


def non_maximal_suppression(values, threshold):
    values.sort()
    output_values = []
    total, count, last_value = values[0], 1, values[0]
    for value in values[1:]:
        if value - last_value > threshold:
            output_values.append(total / count)
            total, count, last_value = value, 1, value
        else:
            total += value
            count += 1
    output_values.append(total / count)
    return output_values


BLUE, RED, BROWN, GRAY = 'blue', 'red', 'brown', 'gray'
RED_LOWER_HSV, RED_UPPER_HSV = (174, 80, 0), (4, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (112, 80, 0), (124, 255, 255)
BROWN_LOWER_HSV, BROWN_UPPER_HSV = (0, 31, 27), (19, 94, 104)
GRAY_LOWER_HSV, GRAY_UPPER_HSV = (66, 3, 121), (126, 69, 210)

MAX_BLOB_ASPECT_RATIO, MIN_BLOB_SIZE = 0.5, 250
MAX_ASPECT_RATIO_ERROR, MIN_RECT_FILL = 0.2, 0.85

OPEN_KERNEL_SIZE, CLOSE_KERNEL_SIZE = 5, 5

MIN_DIMENSION = 360

ACTUAL_RAIL_GAP = 7.5  # in
ACTUAL_GLYPH_SIZE = 6.0  # in
FOV_DEGREES = 58.5  # deg
FOCAL_LENGTH_IN = 3.2 / 25.4  # in

INPUT_DIR = 'cryptobox/images/'
OUTPUT_DIR = 'cryptobox/output/'

OPEN_KERNEL = np.ones((OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE), np.uint8)
CLOSE_KERNEL = np.ones((CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE), np.uint8)

EXTENDED_TRACKING = True

watermark = cv2.imread("watermark.jpg")[:, :, 0]
os.makedirs(OUTPUT_DIR, exist_ok=True)
for i, filename in enumerate(os.listdir(INPUT_DIR)):
    if filename.startswith('.'):
        continue

    base_name = filename.split('.')[0]
    print('processing {}'.format(base_name))

    image = cv2.imread(INPUT_DIR + filename)

    # fill in watermark
    image = cv2.inpaint(image, watermark, 3, cv2.INPAINT_NS)

    output = image.copy()

    # resize and blur
    image = cv2.pyrDown(image)
    image = cv2.pyrDown(image)

    image_height, image_width = image.shape[:2]

    # color conversion and thresholding
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = smart_hsv_range(hsv, RED_LOWER_HSV, RED_UPPER_HSV)
    blue = smart_hsv_range(hsv, BLUE_LOWER_HSV, BLUE_UPPER_HSV)
    brown = smart_hsv_range(hsv, BROWN_LOWER_HSV, BROWN_UPPER_HSV)
    gray = smart_hsv_range(hsv, GRAY_LOWER_HSV, GRAY_UPPER_HSV)

    # pick either red or blue to analyze
    red_count = cv2.countNonZero(red)
    blue_count = cv2.countNonZero(blue)
    color_total = red_count + blue_count

    rails = []

    if color_total != 0:
        if blue_count > red_count:
            rail_image = blue
            output_color = (255, 255, 0)
        else:
            rail_image = red
            output_color = (0, 255, 255)

        # morph color blobs
        rail_morph = cv2.morphologyEx(rail_image, cv2.MORPH_OPEN, OPEN_KERNEL)
        rail_morph = cv2.morphologyEx(rail_morph, cv2.MORPH_CLOSE, CLOSE_KERNEL)

        _, rail_contours, _ = cv2.findContours(rail_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # blobs to rails
        for contour in rail_contours:
            area = int(cv2.contourArea(contour))
            _, _, width, height = cv2.boundingRect(contour)
            if width / height > MAX_BLOB_ASPECT_RATIO or area < MIN_BLOB_SIZE:
                continue
            # centroid calculation
            M = cv2.moments(contour)
            centroid_x = int(M['m10'] / M['m00'])
            rails.append(centroid_x)

    if EXTENDED_TRACKING:
        # find glyphs
        full_glyphs, potential_glyphs = [], []
        glyph_width_sum, glyph_width_count = 0, 0

        for glyph_image, color in ((brown, BROWN), (gray, GRAY)):
            glyph_morph = cv2.morphologyEx(glyph_image, cv2.MORPH_OPEN, OPEN_KERNEL)
            glyph_morph = cv2.morphologyEx(glyph_morph, cv2.MORPH_CLOSE, CLOSE_KERNEL)

            glyph_contours = cv2.findContours(glyph_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

            # find full glyphs from contours
            for contour in glyph_contours:
                area = cv2.contourArea(contour)
                x, y, width, height = cv2.boundingRect(contour)
                rect_area = width * height
                aspect_ratio = height / width
                aspect_ratio_int = round(aspect_ratio)
                if area / rect_area > MIN_RECT_FILL:
                    if abs(aspect_ratio_int - aspect_ratio) < MAX_ASPECT_RATIO_ERROR:
                        if aspect_ratio_int > 1:
                            height /= aspect_ratio_int
                        glyph_width_sum += width
                        glyph_width_count += 1
                        for j in range(aspect_ratio_int):
                            full_glyphs.append((x, y, width, height))
                            y += height
                    else:
                        potential_glyphs.append((x, y, width, height))

        # find partial glyphs
        mean_glyph_size = None
        partial_glyphs = []

        if glyph_width_count > 0:
            mean_glyph_size = glyph_width_sum / glyph_width_count

            for rect in potential_glyphs:
                if rect[0] != 0 and rect[1] != 0 and (rect[0] + rect[2]) != image_width and \
                                (rect[1] + rect[3]) != image_height:
                    continue

                width_ratio = rect[2] / mean_glyph_size
                width_ratio_int = round(width_ratio)
                height_ratio = rect[3] / mean_glyph_size
                height_ratio_int = round(height_ratio)

                if abs(width_ratio - width_ratio_int) < MAX_ASPECT_RATIO_ERROR \
                        or abs(height_ratio - height_ratio_int) < MAX_ASPECT_RATIO_ERROR:
                    partial_glyphs.append(rect)
                    cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                                  (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 127, 255), 10)
                else:
                    cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                                  (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 0, 0), 10)

        # find rails from full glyphs
        if len(full_glyphs) > 0:
            glyph_rails = []
            for x, y, width, height in full_glyphs:
                glyph_center = x + width / 2
                rail_gap = width * ACTUAL_RAIL_GAP / ACTUAL_GLYPH_SIZE
                left_rail = glyph_center - 0.5 * rail_gap
                right_rail = glyph_center + 0.5 * rail_gap
                glyph_rails.extend([left_rail, right_rail])
            glyph_rails = non_maximal_suppression(glyph_rails, mean_glyph_size / 4)
            rails.extend(glyph_rails)
            rails = non_maximal_suppression(rails, 3 * mean_glyph_size / 8)

        for rect in full_glyphs:
            cv2.rectangle(output, (int(4 * rect[0]), int(4 * rect[1])),
                          (int(4 * (rect[0] + rect[2])), int(4 * (rect[1] + rect[3]))), (0, 255, 0), 10)

    for rail in rails:
        cv2.line(output, (int(4 * rail), 0), (int(4 * rail), 4 * image_height), output_color, 10)

    # distance/offset calculation
    if len(rails) > 1:
        rail_gap = (max(rails) - min(rails)) / (len(rails) - 1)
        focal_length_px = (image_width * 0.5) / tan(FOV_DEGREES * 0.5 * pi / 180)
        distance = (ACTUAL_RAIL_GAP * focal_length_px) / rail_gap
        if len(rails) == 4:
            center = sum(rails) / len(rails)
            dx = ((0.5 * image_width - center) * distance) / focal_length_px

            cv2.putText(output, '{:.2f} in, {:.2f} in'.format(distance, dx), (5, 50), cv2.FONT_HERSHEY_DUPLEX, 2,
                        (0, 0, 255), 4)
        else:
            cv2.putText(output, '{:.2f} in'.format(distance), (5, 50), cv2.FONT_HERSHEY_DUPLEX, 2,
                        (0, 0, 255), 4)

    cv2.imwrite('{}/{}.jpg'.format(OUTPUT_DIR, base_name), output)