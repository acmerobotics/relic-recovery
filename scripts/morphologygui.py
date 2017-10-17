import cv2
import numpy as np
import glob
from util import smart_hsv_range

LOWER_HSV, UPPER_HSV = (112, 80, 0), (124, 255, 255)

IMAGE_GLOB = 'cryptobox/images/blue*.jpg'
MIN_DIMENSION = 360


def nothing():
    pass


cv2.namedWindow("image")

cv2.createTrackbar("open_size", "image", 0, 25, nothing)
cv2.createTrackbar("close_size", "image", 0, 25, nothing)

for filename in glob.glob(IMAGE_GLOB):
    image = cv2.imread(filename)

    height, width = image.shape[:2]
    if height < width:
        new_height = MIN_DIMENSION
        new_width = int(width / height * new_height)
    else:
        new_width = MIN_DIMENSION
        new_height = int(height / width * new_width)
    image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = smart_hsv_range(hsv, LOWER_HSV, UPPER_HSV)

    while True:
        open_size = 1 + 2 * cv2.getTrackbarPos("open_size", "image")
        close_size = 1 + 2 * cv2.getTrackbarPos("close_size", "image")

        open_kernel = np.ones((open_size, open_size), np.uint8)
        close_kernel = np.ones((close_size, close_size), np.uint8)

        morphed_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_CLOSE, close_kernel)

        cv2.imshow("image", morphed_mask)
        k = cv2.waitKey(1000) & 0xFF
        if k == 27:
            break

cv2.destroyAllWindows()