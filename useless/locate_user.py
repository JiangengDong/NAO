from __future__ import print_function
from __future__ import division
import cv2
import math
import numpy as np

IMGNo = 0
# intrinsic parameters
# 0:Top, 1:bottom
with np.load('camera0.npz') as X:
    mtx_0, dist_0, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    mtx_0 = np.matrix(mtx_0)
with np.load('camera1.npz') as X:
    mtx_1, dist_1, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    mtx_1 = np.matrix(mtx_1)


def locate2d(img):
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        upper_color = np.array([255, 255, 255])
        lower_color = np.array([10, 60, 50])  # 0, 90, 130
        mask = cv2.inRange(hsv, lower_color, upper_color)
        # cv2.imshow('mask',mask);cv2.waitKey(0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        res = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        # cv2.imshow('mask1',mask);cv2.waitKey(0)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 200,
                                   param1=100, param2=10, minRadius=0, maxRadius=200)[0]
        return_x = []
        return_y = []
        # i = circles[np.lexsort(-circles.T)][0]
        for i in circles:
            cv2.circle(img, (i[0], i[1]), i[2], (0, 0, 255), 2)
            cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
            return_x.append(i[0])
            return_y.append(i[1])
        return return_x[0], return_y[0]
    except Exception:
        return None


def locate3d(img1, img2, pitch0=-8 / 180 * math.pi):
    # parameter for camera
    f_y = 271.8194
    f_x = 271.7887

    n_in = 0  # number of valid pictures
    x, y = None, None
    p1 = locate2d(img1)
    p2 = locate2d(img2)
    pitch1, yaw1, pitch2, yaw2 = None, None, None, None
    # if p1 is not None:
    #     pitch1 = math.atan((p1[1] - 120) / f_y) + 0.02094395 + pitch0
    #     yaw1 = math.atan((160 - p1[0]) / f_x)
    #     n_in += 1
    if p2 is not None:
        pitch2 = math.atan((p2[1] - 120) / f_y) + 0.6928957 + pitch0
        yaw2 = math.atan((160 - p2[0]) / f_x)
        # n_in += 1

    # if n_in is 2:
    #     deltaz = (63.64 * math.cos(pitch0) + 58.71 * math.sin(pitch0)) \
    #              -(17.74 * math.cos(pitch0) + 50.71 * math.sin(pitch0))
    #     deltax = (58.71 * math.cos(pitch0) - 63.64 * math.sin(pitch0)) \
    #              -(50.71 * math.cos(pitch0) - 17.74 * math.sin(pitch0))
    #     x = (deltaz + math.tan(pitch1) * deltax) / (math.tan(pitch1) - math.tan(pitch2))
    #     y = x * math.tan((yaw1 + yaw2) / 2)
    #     z = 478.23 - x * math.tan(pitch2)

    if n_in is 0:
        print('Cannot locate.')

    else:
        # if p1 is not None:
        #     x = (459.59 + 63.64 * math.cos(pitch0) + 58.71 * math.sin(pitch0)) / math.tan(pitch1) \
        #         + 58.71 * math.cos(pitch0) - 63.64 * math.sin(pitch0)
        #     y = x * math.tan(yaw1)
        #     z = 0
        if p2 is not None:
            x = (459.59 + 17.74 * math.cos(pitch0) + 50.71 * math.sin(pitch0)) / math.tan(pitch2) \
                + 50.71 * math.cos(pitch0) - 17.74 * math.sin(pitch0)
            y = x * math.tan(yaw2)

    print('p0 =', p1, ', p1 =', p2, ', x =', x, ', y =', y)
    return x, y


def main():
    img0 = cv2.imread('0.bmp', 1)
    img1 = cv2.imread('1.bmp', 1)
    x, y = locate3d(img0, img1, -7 / 180 * math.pi)
    return x, y


if __name__ == "__main__":
    main()
