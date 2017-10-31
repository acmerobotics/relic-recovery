import cv2
import numpy as np
from matplotlib import pyplot as plt
from util import resize_min_dim, smart_hsv_range

IMAGE_FILENAME = 'jewel/blue_jewel.jpg'
MIN_DIMENSION = 480
# LOWER_HSV, UPPER_HSV = (170, 80, 0), (7, 255, 255)
LOWER_HSV, UPPER_HSV = (95, 80, 10), (112, 255, 255)

image = cv2.imread(IMAGE_FILENAME)

image = resize_min_dim(image, MIN_DIMENSION)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

mask = smart_hsv_range(hsv, LOWER_HSV, UPPER_HSV)

hue_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])

plt.gcf().canvas.set_window_title(IMAGE_FILENAME)

plt.subplot(2, 3, 1)
plt.plot(hue_hist)
plt.xlim([0, 180])
plt.title('Hue Histogram')

plt.subplot(2, 3, 2)
plt.imshow(hsv[:,:,0], cmap=plt.cm.binary)
plt.title('Hue')

plt.subplot(2, 3, 4)
plt.imshow(hsv[:,:,1], cmap=plt.cm.binary)
plt.title('Saturation')

plt.subplot(2, 3, 5)
plt.imshow(hsv[:,:,2], cmap=plt.cm.binary)
plt.title('Value')

plt.subplot(2, 3, 3)
plt.imshow(mask, cmap=plt.cm.binary)
plt.title('Mask')

mask_3c = np.zeros(image.shape, np.uint8)
for i in range(3):
    mask_3c[:,:,i] = mask
plt.subplot(2, 3, 6)
plt.imshow(cv2.cvtColor(cv2.bitwise_and(image, mask_3c), cv2.COLOR_BGR2RGB))
plt.title('Image')

plt.show()
