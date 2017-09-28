import cv2
import numpy as np
from matplotlib import pyplot as plt

IMAGE_FILENAME = 'cryptobox/red004.jpg'
MIN_DIMENSION = 480
LOWER_HSV, UPPER_HSV = (170, 80, 0), (7, 255, 255)

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
if LOWER_HSV[0] > UPPER_HSV[0]:
    mask = cv2.inRange(hsv, LOWER_HSV, (180, UPPER_HSV[1], UPPER_HSV[2]))
    mask2 = cv2.inRange(hsv, (0, LOWER_HSV[1], LOWER_HSV[2]), UPPER_HSV)
    cv2.bitwise_or(mask, mask2, mask)
else:
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

hue_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])

plt.gcf().canvas.set_window_title(IMAGE_FILENAME)

plt.subplot(2, 3, 1)
plt.plot(hue_hist)
plt.xlim([0, 180])
plt.title('Hue Histogram')

HSV_TITLES = ['Hue', 'Saturation', 'Value']
for i in range(3):
    plt.subplot(2, 3, i + 2)
    plt.imshow(hsv[:,:,i], cmap=plt.cm.binary)
    plt.title(HSV_TITLES[i])

plt.subplot(2, 3, 5)
plt.imshow(mask, cmap=plt.cm.binary)
plt.title('Mask')

mask_3c = np.zeros(image.shape, np.uint8)
for i in range(3):
    mask_3c[:,:,i] = mask
plt.subplot(2, 3, 6)
plt.imshow(cv2.cvtColor(cv2.bitwise_and(image, mask_3c), cv2.COLOR_BGR2RGB))
plt.title('Image')

plt.show()