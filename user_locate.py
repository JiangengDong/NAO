from __future__ import print_function
from __future__ import division
import cv2
import math
import numpy as np


def locate2d(img):
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv', hsv)
        cv2.waitKey(0)
        upper_color = np.array([35, 255, 255])
        lower_color = np.array([10, 50, 150])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        #cv2.imshow('mask',mask);cv2.waitKey(0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        res = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        #print gray.shape
        # cv2.imshow('mask1',mask);cv2.waitKey(0)
        edges = cv2.Canny(gray, 100, 200)
        # cv2.imshow('edge',edges)
        # cv2.waitKey(2000)
        cv2.destroyWindow("edge")
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 2, 200,
                                   param1=100, param2=10, minRadius=0, maxRadius=200)[0]
        return_x = []
        return_y = []
        # i = circles[np.lexsort(-circles.T)][0]
        if circles is not None:
            for i in circles:
                cv2.circle(img, (i[0], i[1]), i[2], (0, 0, 255), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
                return_x.append(i[0])
                return_y.append(i[1])
            return return_x[0], return_y[0]
        else:
            rows = gray.shape[0]
            cols = gray.shape[1]
            loca_x = []
            loca_y = []
            for i in range(0,rows-1):
                for j in range(0,cols-1):
                    if gray[i,j] != 0:
                        loca_x.append(i)
                        loca_y.append(j)
            loca_x = np.array(loca_x,dtype='int')
            loca_y = np.array(loca_y, dtype='int')
            x = np.mean(loca_x)
            y = np.mean(loca_y)
            return x,y
    except Exception:
        return None


def locate3d(img2, yaw0, pitch0):
    # parameter for camera
    f_y = 271.8194
    f_x = 271.7887

    x, y, yaw2, pitch2 = None, None, None, None
    p2 = locate2d(img2)
    if p2 is not None:
        pitch2 = math.atan((p2[1] - 120) / f_y) + 0.6928957 + pitch0
        yaw2 = math.atan((160 - p2[0]) / f_x) + yaw0
        x = (300+126.5 + 17.74 * math.cos(pitch0) + 50.71 * math.sin(pitch0)) / math.tan(pitch2) \
            + 50.71 * math.cos(pitch0) - 17.74 * math.sin(pitch0)
        y = x * math.tan(yaw2)
        x = x/1000.0
        y = y/1000.0
    else:
        print('Cannot locate.')

    print('p =', p2, ', x =', x, ', y =', y)
    return x, y, yaw2, pitch2


def main():
    img1 = cv2.imread('1.bmp', 1)
    x, y = locate3d(img1, -7 / 180 * math.pi)
    return x, y


if __name__ == "__main__":
    main()
