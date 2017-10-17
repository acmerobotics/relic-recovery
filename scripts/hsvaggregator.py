import cv2
import numpy as np
from matplotlib import pyplot as plt
import glob
from util import resize_min_dim

IMAGE_GLOB = '/Users/ryanbrott/Desktop/ACME/FpsVision-images-1507076845270/1*.jpg'
MIN_DIMENSION = 480


aggregate_hist = np.zeros((180, 1), np.float32)
filenames = glob.glob(IMAGE_GLOB)
for filename in filenames:
    print(filename)

    image = cv2.imread(filename)

    image = resize_min_dim(image, MIN_DIMENSION)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    hue_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])

    aggregate_hist += hue_hist

plt.gcf().canvas.set_window_title("{} [{}]".format(IMAGE_GLOB, len(filenames)))

plt.plot(aggregate_hist)
plt.xlim([0, 180])
plt.title('Hue Histogram')
plt.show()