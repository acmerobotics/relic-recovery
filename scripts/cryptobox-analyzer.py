import cv2
import numpy as np
import os

RED_LOWER_HSV, RED_UPPER_HSV = (0, 80, 0), (7, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (110, 80, 0), (120, 255, 255)

KERNEL_SIZE = 11

# IMAGE_FILENAME = 'cryptobox/blue001.jpg'
IMAGE_DIR = 'cryptobox/'
WIDTH, HEIGHT = 576, 480 # 4:3 aspect ratio

os.makedirs('cryptobox2/', exist_ok=True)
for filename in os.listdir(IMAGE_DIR):
    if filename.startswith('red'):
        lower_hsv, upper_hsv = RED_LOWER_HSV, RED_UPPER_HSV
    else:
        lower_hsv, upper_hsv = BLUE_LOWER_HSV, BLUE_UPPER_HSV

    image = cv2.imread(IMAGE_DIR + filename)
    image = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_CUBIC)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
    morphed_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

    im2, contours, hierarchy = cv2.findContours(morphed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # output = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    # output[:,:,0] = morphed_mask
    # output[:,:,1] = morphed_mask
    # output[:,:,2] = morphed_mask

    for contour in contours:
        rect = cv2.boundingRect(contour)
        if (rect[2] / rect[3]) > 0.35:
            cv2.drawContours(image, [contour], 0, (255, 0, 255), 3)
            continue
        cv2.drawContours(image, [contour], 0, (0, 255, 255), 3)
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((HEIGHT - x) * vy / vx) + y)
        cv2.line(image, (HEIGHT - 1, righty), (0, lefty), (0, 255, 0), 2)

    # for contour in contours:
    #     epsilon = 0.01 * cv2.arcLength(contour, True)
    #     approx = cv2.approxPolyDP(contour, epsilon, True)
    #     cv2.drawContours(image, [approx], 0, (255, 255, 0), 3)

    cv2.imwrite('cryptobox2/' + filename, image)