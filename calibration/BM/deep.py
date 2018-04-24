import numpy as np
import cv2
from matplotlib import pyplot as plt

imgR = cv2.imread('testL.bmp',0)
imgL = cv2.imread('testR.bmp',0)

stereo = cv2.StereoBM_create(32, 5)
disparity = stereo.compute(imgL, imgR)

plt.imshow(disparity,'gray')
plt.show()
