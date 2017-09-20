import cv2
from matplotlib import pyplot as plt

IMAGE_FILENAME = 'cryptobox/blue001.jpg'

image = cv2.imread(IMAGE_FILENAME)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
hue_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])

plt.subplot(2, 2, 1)
plt.plot(hue_hist)
plt.xlim([0, 180])
plt.title('Hue Histogram')

HSV_TITLES = ['Hue', 'Saturation', 'Value']
for i in range(3):
    plt.subplot(2, 2, i + 2)
    plt.imshow(hsv[:,:,i], cmap=plt.cm.binary)
    plt.title(HSV_TITLES[i])

plt.show()