import cv2
import numpy as np
from matplotlib import pyplot as plt

RED_LOWER_HSV, RED_UPPER_HSV = (0, 80, 0), (7, 255, 255)
RED2_LOWER_HSV, RED2_UPPER_HSV = (170, 80, 0), (180, 255, 255)
BLUE_LOWER_HSV, BLUE_UPPER_HSV = (110, 80, 0), (130, 255, 255)
IMAGE_FILENAME = 'cryptobox/blue002.jpg'
MIN_DIMENSION = 480
KERNEL_SIZES = (0, 5, 9, 13)
NUM_KERNELS = len(KERNEL_SIZES)

image = cv2.imread(IMAGE_FILENAME)
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

for title, mask in zip(('Red', 'Blue'), (red, blue)):
    plt.gcf().canvas.set_window_title('{} - {}'.format(IMAGE_FILENAME, title))
    for i, open_size in enumerate(KERNEL_SIZES):
        open_kernel = np.ones((open_size, open_size), np.uint8)
        for j, close_size in enumerate(KERNEL_SIZES):
            close_kernel = np.ones((close_size, close_size), np.uint8)
            if open_size == 0:
                morphed_mask = mask.copy()
            else:
                morphed_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            if close_size != 0:
                morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_CLOSE, close_kernel)

            plt.subplot(NUM_KERNELS, NUM_KERNELS, i + NUM_KERNELS * j + 1)
            plt.axis('off')
            plt.imshow(morphed_mask, cmap=plt.cm.binary)
            plt.title('Open {0}x{0}, Close {1}x{1}'.format(open_size, close_size))
    plt.show()