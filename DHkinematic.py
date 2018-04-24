from __future__ import print_function
import math
import numpy as np
import time


def partial_y(y, x):
    return 1 / (x + y ** 2 / x)


def partial_x(y, x):
    return -1 / (x ** 2 / y + y)


def kinematic(theta, diff=False):
    costheta = [math.cos(t) for t in theta]
    sintheta = [math.sin(t) for t in theta]
    d = [0, 0, 0, 0, 0, 0]
    a = [0, 0, -100, -102.9, 0, 45.19]
    alpha = [math.pi / 2, math.pi / 2, 0, 0, math.pi / 2, 0]
    cosalpha = [math.cos(t) for t in alpha]
    sinalpha = [math.sin(t) for t in alpha]

    T = [None] * 6
    T_l = np.eye(4)
    for i in range(6):
        T[i] = np.array([[costheta[i], -cosalpha[i] * sintheta[i], sinalpha[i] * sintheta[i], a[i] * costheta[i]],
                         [sintheta[i], cosalpha[i] * costheta[i], -sinalpha[i] * costheta[i], a[i] * sintheta[i]],
                         [0, sinalpha[i], cosalpha[i], d[i]],
                         [0, 0, 0, 1]])
        T_l = np.dot(T_l, T[i])
    print("T: ", T_l)
    pos = np.array(T_l[0:3, 3]).reshape([3])
    nvec = np.array(T_l[0:3, 0]).reshape([3])
    svec = np.array(T_l[0:3, 1]).reshape([3])
    avec = np.array(T_l[0:3, 2]).reshape([3])

    y = math.atan2(nvec[1], nvec[0])
    cosy = math.cos(y)
    siny = math.sin(y)
    p = math.atan2(-nvec[2], cosy * nvec[2] + siny * nvec[1])
    r = math.atan2(siny * avec[0] - cosy * avec[1], -siny * svec[0] + cosy * svec[1])
    rot = np.array([r, p, y])
    Jacob = None

    if diff:
        dT = [np.eye(4)] * 6
        for i in range(6):
            for j in range(6):
                if j == i:
                    dT[j] = np.dot(dT[j], np.array(
                        [[-sintheta[i], -cosalpha[i] * costheta[i], sinalpha[i] * costheta[i], -a[i] * sintheta[i]],
                         [costheta[i], -cosalpha[i] * sintheta[i], sinalpha[i] * sintheta[i], a[i] * costheta[i]],
                         [0, 0, 0, 0],
                         [0, 0, 0, 1]]))
                else:
                    dT[j] = np.dot(dT[j], T[i])
        Jacob = np.zeros([6, 6])
        dy_ny = partial_y(nvec[1], nvec[0])
        dy_nx = partial_x(nvec[1], nvec[0])
        dp_1 = partial_y(-nvec[2], cosy * nvec[2] + siny * nvec[1])
        dp_2 = partial_x(-nvec[2], cosy * nvec[2] + siny * nvec[1])
        dp_nz = dp_1 * (-1) + dp_2 * cosy
        dp_ny = dp_2 * siny
        dp_y = dp_2 * (-siny * nvec[2] + cosy * nvec[1])
        dr_1 = partial_y(siny * avec[0] - cosy * avec[1], -siny * svec[0] + cosy * svec[1])
        dr_2 = partial_x(siny * avec[0] - cosy * avec[1], -siny * svec[0] + cosy * svec[1])
        dr_y = dr_1 * (cosy * avec[0] + siny * avec[1]) + dr_2 * (-cosy * svec[0] - siny * svec[1])
        dr_ax = dr_1 * siny
        dr_ay = dr_1 * (-cosy)
        dr_sx = dr_2 * (-siny)
        dr_sy = dr_2 * cosy
        for i in range(6):
            Jacob[0:3, i] = dT[i][0:3, 3]
            Jacob[5, i] = dy_ny * dT[i][1, 0] + dy_nx * dT[i][0, 0]
            Jacob[4, i] = dp_nz * dT[i][2, 0] + dp_ny * dT[i][1, 0] + dp_y * Jacob[5, i]
            Jacob[3, i] = (dr_y * Jacob[5, i] + dr_ax * dT[i][0, 2] + dr_ay * dT[i][1, 2]
                           + dr_sx * dT[i][0, 1] + dr_sy * dT[i][1, 1])

    return pos, rot, np.matrix(Jacob)


if __name__ == '__main__':
    theta = [math.pi/2, math.pi*5/4, 0, 0, math.pi, 0]
    start_time = time.time()
    pos, rot, Jacob = kinematic(theta)
    end_time = time.time()
    print('Time: ', end_time - start_time)
    print('Pos: ', pos)
    print('Rot: ', rot)
    print('Jacob: ', Jacob)
