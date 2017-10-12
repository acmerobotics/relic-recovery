import cv2
import numpy as np
import os
from util import resize_min_dim, smart_hsv_range

INPUT_DIR = "cryptobox/images/"
OUTPUT_DIR = 'cryptobox/output/'
MIN_DIMENSION = 480

IMAGE_WIN = "image"
OUTPUT_WIN = "output"


def nothing(x):
    pass


# cv2.createTrackbar("HL", IMAGE_WIN, 0, 180, nothing)
# cv2.createTrackbar("HU", IMAGE_WIN, 0, 180, nothing)
# cv2.createTrackbar("SL", IMAGE_WIN, 0, 255, nothing)
# cv2.createTrackbar("SU", IMAGE_WIN, 0, 255, nothing)
# cv2.createTrackbar("VL", IMAGE_WIN, 0, 255, nothing)
# cv2.createTrackbar("VU", IMAGE_WIN, 0, 255, nothing)
#
# cv2.setTrackbarPos("HU", IMAGE_WIN, 180)
# cv2.setTrackbarPos("SU", IMAGE_WIN, 255)
# cv2.setTrackbarPos("VU", IMAGE_WIN, 255)

hsv_values = []


def save_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        hsv_values.append([int(i) for i in hsv[y // 2,x // 2,:]])


for filename in os.listdir(INPUT_DIR):
    if filename.startswith("."):
        continue

    image = cv2.imread(INPUT_DIR + filename)
    print(image.shape)

    # image = resize_min_dim(image, MIN_DIMENSION)
    image = cv2.pyrDown(image)
    image = cv2.pyrDown(image)
    print(image.shape)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.namedWindow(IMAGE_WIN)
    cv2.setMouseCallback(IMAGE_WIN, save_hsv_value)

    cv2.namedWindow(OUTPUT_WIN)

    while True:
        # hueLower = cv2.getTrackbarPos("HL", IMAGE_WIN)
        # satLower = cv2.getTrackbarPos("SL", IMAGE_WIN)
        # valLower = cv2.getTrackbarPos("VL", IMAGE_WIN)
        #
        # hueUpper = cv2.getTrackbarPos("HU", IMAGE_WIN)
        # satUpper = cv2.getTrackbarPos("SU", IMAGE_WIN)
        # valUpper = cv2.getTrackbarPos("VU", IMAGE_WIN)

        if len(hsv_values) > 1:
            hueLower = min(hsv_values, key=lambda x: x[0])[0]
            satLower = min(hsv_values, key=lambda x: x[1])[1]
            valLower = min(hsv_values, key=lambda x: x[2])[2]
            hueUpper = max(hsv_values, key=lambda x: x[0])[0]
            satUpper = max(hsv_values, key=lambda x: x[1])[1]
            valUpper = max(hsv_values, key=lambda x: x[2])[2]

            for hue, _, _ in hsv_values:
                if hue != hueLower and hue != hueUpper and (hue < hueLower or hue > hueUpper):
                    hueLower, hueUpper = hueUpper, hueLower
                    break

            # if (hueLower + 180 - hueUpper) < (hueUpper - hueLower):
            #     hueLower, hueUpper = hueUpper, hueLower

            lower_hsv = (hueLower, satLower, valLower)
            upper_hsv = (hueUpper, satUpper, valUpper)

            print("Hue Ranges:")
            print(lower_hsv)
            print(upper_hsv)

            mask = smart_hsv_range(hsv, lower_hsv, upper_hsv)
        else:
            mask = np.zeros(image.shape, np.uint8)

        # mask_3c = np.zeros(image.shape, np.uint8)
        # mask_3c[:,:,0] = mask
        # mask_3c[:,:,1] = mask
        # mask_3c[:,:,2] = mask
        # output = cv2.bitwise_and(image, mask_3c)

        cv2.imshow(IMAGE_WIN, cv2.resize(image, (2 * image.shape[1], 2 * image.shape[0])))
        cv2.imshow(OUTPUT_WIN, cv2.resize(mask, (2 * image.shape[1], 2 * image.shape[0])))
        k = cv2.waitKey(200) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("c"):
            hsv_values = []
        elif k == ord("p") and len(hsv_values) > 0:
            hsv_values.pop()
        elif k == ord("w"):
            base_name = filename.split('.')[0]
            with open('{}/{}_thresholds.txt'.format(OUTPUT_DIR, base_name), mode='w') as fh:
                fh.write(str(lower_hsv) + '\n')
                fh.write(str(upper_hsv) + '\n')

cv2.destroyAllWindows()