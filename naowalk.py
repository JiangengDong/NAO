# encoding: utf-8
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import math
import time
import numpy as np
import matplotlib.pyplot as plt

N = 50


def get_com_locus(theta):
    s0 = 0.04
    s_x = s0 * math.cos(theta)
    s_y = s0 * math.sin(theta)
    s_y_max = s_y / 2 + 0.12
    s_y_min = -s_y / 2 + 0.12
    z = 0.3
    g = 9.8
    T_c = math.sqrt(z / g)
    T = 1

    vx_0 = s_x / 2 * (math.cosh(T / T_c) + 1) / (T_c * math.sinh(T / T_c))
    vy_0 = (s_y_min / 2 - s_y_max / 2 * math.cosh(T / T_c)) / (T_c * math.sinh(T / T_c))

    def xfun(t):
        return - s_x / 2 * math.cosh(t / T_c) + T_c * vx_0 * math.sinh(t / T_c)

    def yfun(t):
        return s_y_max / 2 * math.cosh(t / T_c) + T_c * vy_0 * math.sinh(t / T_c)

    # locus for steps with even number
    comx_b1 = [0.0] * N
    comy_b1 = [0.0] * N
    t_b1 = [0.0] * N
    for i in range(N):
        t_b1[i] = (i + 1) * T / (N - 1)
        comx_b1[i] = xfun(t_b1[i] - T / (N - 1))
        comy_b1[i] = yfun(t_b1[i] - T / (N - 1))
    locus_b1 = (comx_b1, comy_b1, t_b1)

    # Locus for steps with odd number
    comx_b2 = comx_b1
    comy_b2 = comy_b1[-1::-1]
    t_b2 = t_b1
    locus_b2 = (comx_b2, comy_b2, t_b2)

    # locus for the first step
    comx_s = [0.0] * N * 2
    comy_s = [0.0] * N * 2
    t_s = [0.0] * N * 2

    def yfun_s(t):
        return (-vy_0 / (2 * T) ** 6 - 3 * s_y / 2 / (2 * T) ** 7) * t ** 7 + (
                3.5 * s_y / 2 / (2 * T) ** 6 + vy_0 / (2 * T) ** 5) * t ** 6 + 0.06

    def xfun_s(t):
        if t > T:
            return s_x / 2 / math.cosh(T / T_c) * math.cosh((t - T) / T_c)
        else:
            return s_x / 2 / math.cosh(T / T_c) * (t / T) ** 2

    for i in range(N * 2):
        t_s[i] = (i + 1) * 2 * T / (2 * N - 1)
        comy_s[i] = yfun_s(t_s[i] - 2 * T / (2 * N - 1))
        comx_s[i] = xfun_s(t_s[i] - 2 * T / (2 * N - 1))
    locus_s = (comx_s, comy_s, t_s)

    # locus for the last step
    comx_e = [-x for x in comx_s][-1::-1]
    comy_e = comy_s[-1::-1]
    locus_e = (comx_e, comy_e, t_s)

    return locus_b1, locus_b2, locus_s, locus_e
    pass


def q2p(q, is_right=False):
    l1 = 0.1
    l2 = 0.1029
    if is_right:
        q[1] = -q[1]
        q[5] = -q[5]
    r = math.sqrt(l1 ** 2 + l2 ** 2 + 2 * l1 * l2 * math.cos(q[3]))
    x = -r * math.sin(q[2] + math.acos((l1 ** 2 + r ** 2 - l2 ** 2) / 2 / r / l1))
    rho = math.sqrt(r ** 2 - x ** 2)
    y = rho * math.sin(q[1]) + 0.05
    z = rho * math.cos(q[1]) + 0.04519 + 0.085
    return [x, y, z, 0, 0, 0]


def p2q(p, is_right=False):  # 根据末端位置计算关节角
    q = [0] * 6
    x, y, z, _, _, _ = p
    y = y - 0.05
    z = z - 0.04519 - 0.085
    r = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    l1 = 0.1
    l2 = 0.1029
    q[1] = math.atan(y / z)
    q[5] = -q[1] * 1.05
    q[3] = math.pi - math.acos((l1 ** 2 + l2 ** 2 - r ** 2) / 2 / l1 / l2)
    q[2] = -math.asin(x / r) - math.acos((l1 ** 2 + r ** 2 - l2 ** 2) / 2 / r / l1)
    q[4] = -q[3] - q[2]
    if is_right is True:
        q[1] = -q[1]
        q[5] = -q[5]
    return q


class Move(object):
    def __init__(self, LLeg, RLeg, motion_proxy):
        self.LLeg = LLeg
        self.RLeg = RLeg
        self.motion_proxy = motion_proxy

    def forward(self, r, theta, headlock=None, is_permanent=False):
        LLeg = self.LLeg
        RLeg = self.RLeg
        motion_proxy = self.motion_proxy
        l_b1, l_b2, l_s, l_e = get_com_locus(theta)  # 获取质心运动轨迹

        # 将质心运动轨迹转换成支撑腿的关节角
        x_b1, y_b1, t_b1 = l_b1
        q_b1_l = [None] * N
        q_b1_r = [None] * N
        for i in range(N):
            q_b1_r[i] = p2q([0 - x_b1[i], y_b1[i], 0.3, 0, 0, 0], True)
            q_b1_l[i] = p2q([0 - x_b1[i], y_b1[i], 0.3, 0, 0, 0])

        x_b2, y_b2, t_b2 = l_b2
        q_b2_l = [None] * N
        q_b2_r = [None] * N
        for i in range(N):
            q_b2_r[i] = p2q([0 - x_b2[i], y_b2[i], 0.3, 0, 0, 0], True)
            q_b2_l[i] = p2q([0 - x_b2[i], y_b2[i], 0.3, 0, 0, 0])

        x_s, y_s, t_s = l_s
        q_s_l = [None] * 2 * N
        q_s_r = [None] * 2 * N
        for i in range(2 * N):
            q_s_r[i] = p2q([0 - x_s[i], y_s[i], 0.3, 0, 0, 0], True)
            q_s_l[i] = p2q([0 - x_s[i], y_s[i], 0.3, 0, 0, 0])

        x_e, y_e, t_e = l_e
        q_e_l = [None] * 2 * N
        q_e_r = [None] * 2 * N
        for i in range(2 * N):
            q_e_r[i] = p2q([0 - x_e[i], y_e[i], 0.3, 0, 0, 0], True)
            q_e_l[i] = p2q([0 - x_e[i], y_e[i], 0.3, 0, 0, 0])

        # 保存转换后的关节角
        if is_permanent:
            x = [0]
            x_n = 0
            y = [0]
            y_n = 0

            x = x + [x_n + x - x_s[0] for x in x_s]
            y = y + [y_n + y - y_s[0] for y in y_s]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b1[0] for x in x_b1]
            y = y + [y_n - y + y_b1[0] for y in y_b1]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b2[0] for x in x_b2]
            y = y + [y_n + y - y_b2[0] for y in y_b2]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b1[0] for x in x_b1]
            y = y + [y_n - y + y_b1[0] for y in y_b1]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b2[0] for x in x_b2]
            y = y + [y_n + y - y_b2[0] for y in y_b2]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b1[0] for x in x_b1]
            y = y + [y_n - y + y_b1[0] for y in y_b1]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_b2[0] for x in x_b2]
            y = y + [y_n + y - y_b2[0] for y in y_b2]
            x_n = x[-1]
            y_n = y[-1]

            x = x + [x_n + x - x_e[0] for x in x_e]
            y = y + [y_n - y + y_e[0] for y in y_e]
            x_n = x[-1]
            y_n = y[-1]

            np.savez("Locus/Locus_b1.npz", q_l=q_b1_l, t=t_b1, q_r=q_b1_r)
            plt.plot(t_b1, x_b1, t_b1, y_b1)
            plt.show()

            np.savez("Locus/Locus_b2.npz", q_l=q_b2_l, t=t_b1, q_r=q_b2_r)
            plt.plot(t_b2, x_b2, t_b2, y_b2)
            plt.show()

            np.savez("Locus/Locus_s.npz", q_l=q_s_l, t=t_s, q_r=q_s_r)
            plt.plot(t_s, x_s, t_s, y_s)
            plt.show()

            np.savez("Locus/Locus_e.npz", q_l=q_e_l, t=t_e, q_r=q_e_r)
            plt.plot(t_e, x_e, t_e, y_e)
            plt.show()

            plt.plot(range(len(x)), x, range(len(y)), y)
            plt.show()
            pass

        T = 1 * 2
        latency = 0.01
        s0 = 0.04
        s_x = s0 * math.cos(theta)
        s_y = 0.12
        s_y_max = s0 * math.sin(theta) / 2 + 0.12
        s_y_min = -s0 * math.sin(theta) / 2 + 0.12

        # 初始化
        LLeg.set_stiffness(1.0)
        RLeg.set_stiffness(1.0)
        id = LLeg.set_joint([p2q([0, s_y / 2, 0.30, 0, 0, 0])], [2])
        id = RLeg.set_joint([p2q([0, s_y / 2, 0.30, 0, 0, 0], True)], [2])
        headlock()
        motion_proxy.wait(id, 2500)

        # 第一步
        id_l = LLeg.set_joint([p2q([0, s_y / 2, 0.30, 0, 0, 0]),
                               p2q([0, 0.11, 0.30, 0, 0, 0]),
                               p2q([s_x / 4, 0.11, 0.275, 0, 0, 0]),
                               p2q([s_x / 2, s_y_max / 2, 0.30, 0, 0, 0])],
                              [latency, T + latency, T * 1.4 + latency, T * 2.0 + latency])
        id_r = RLeg.set_joint(q_s_r, [t_i * 2 for t_i in t_s])
        if headlock is not None:
            headlock()
        motion_proxy.wait(id_l, 5000)
        motion_proxy.wait(id_r, 5000)

        # 左右腿交替，支撑腿按计算出的轨迹，悬空腿快速移动到需要的位置
        for i in range(int(r // 0.08)):
            id_r = RLeg.set_joint([p2q([-s_x / 2, s_y_max / 2, 0.30, 0, 0, 0], True),
                                   p2q([0, 0.11, 0.275, 0, 0, 0], True),
                                   p2q([s_x / 2, s_y_min / 2, 0.30, 0, 0, 0], True)],
                                  [latency, T / 2 + latency, T + latency])
            id_l = LLeg.set_joint(q_b1_l, [t_i * 2 + 0.05 for t_i in t_b1])
            if headlock is not None:
                headlock()  # 每走一步调节一下头部位置
            motion_proxy.wait(id_l, 5000)
            motion_proxy.wait(id_r, 5000)

            id_l = LLeg.set_joint([p2q([-s_x / 2, s_y_min / 2, 0.30, 0, 0, 0]),
                                   p2q([0, 0.11, 0.275, 0, 0, 0]),
                                   p2q([s_x / 2, s_y_max / 2, 0.30, 0, 0, 0])],
                                  [latency, T / 2 + latency, T + latency])
            id_r = RLeg.set_joint(q_b2_r, [t_i * 2 + 0.05 for t_i in t_b2])
            if headlock is not None:
                headlock()
            motion_proxy.wait(id_l, 5000)
            motion_proxy.wait(id_r, 5000)

        # 最后一步
        id_r = RLeg.set_joint([p2q([-s_x / 2, s_y_max / 2, 0.30, 0, 0, 0], True),
                               p2q([0, 0.11, 0.275, 0, 0, 0], True),
                               p2q([0, 0.11, 0.29, 0, 0, 0], True),
                               p2q([0.00, s_y / 2, 0.30, 0, 0, 0], True)],
                              [latency, T * 0.6 + latency, T + latency, T * 2 + latency])
        id_r = LLeg.set_joint(q_e_l, [t_i * 2 for t_i in t_e])
        if headlock is not None:
            headlock()
        motion_proxy.wait(id_l, 5000)
        motion_proxy.wait(id_r, 5000)

    def turn(self, n, headlock=None):
        LLeg = self.LLeg
        RLeg = self.RLeg
        motion_proxy = self.motion_proxy
        LLeg.set_stiffness(1.0)
        RLeg.set_stiffness(1.0)
        q0 = -math.pi / 180 * 10
        T = 1

        if n > 0:  # 逆时针
            # 站立
            time.sleep(2)
            id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
            id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
            motion_proxy.wait(id, 1500)
            for i in range(n):
                # 移重心
                id = RLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0], True)], [T])
                id = LLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0])], [T])
                motion_proxy.wait(id, 1500)
                # 抬腿
                id = RLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0], True)], [T])
                id = LLeg.set_joint([p2q([0, 0.10, 0.25, 0, 0, 0])], [T])
                motion_proxy.wait(id, 1500)
                # 转腿
                motion_proxy.angleInterpolation(['LHipYawPitch'], [q0], [T], True)
                # 放腿
                q = p2q([0, -0.02, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                q = p2q([0, 0.08, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)

                q = p2q([0, 0.05, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                q = p2q([0, 0.05, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                headlock()
                # 移重心
                q = p2q([0, 0.08, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                q = p2q([0, -0.02, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                # 抬腿
                q = p2q([0, 0.10, 0.25, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                q = p2q([0, -0.02, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                # 转腿
                motion_proxy.angleInterpolation(['LHipYawPitch'], [math.pi / 180 * 0], [T], True)
                # 放腿
                id = RLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0], True)], [T])
                id = LLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0])], [T])
                motion_proxy.wait(id, 1500)

                id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
                id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
                motion_proxy.wait(id, 1500)
                headlock()
            id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
            id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
            motion_proxy.wait(id, 1500)
            id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [T])
            id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [T])
            motion_proxy.wait(id, 1500)

        if n < 0:  # 逆时针
            # 站立
            id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
            id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
            motion_proxy.wait(id, 1500)
            for i in range(-1 * n):
                # 移重心
                id = LLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0])], [T])
                id = RLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0], True)], [T])
                motion_proxy.wait(id, 1500)
                # 抬腿
                id = LLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0])], [T])
                id = RLeg.set_joint([p2q([0, 0.10, 0.25, 0, 0, 0], True)], [T])
                motion_proxy.wait(id, 1500)
                # 转腿
                motion_proxy.angleInterpolation(['LHipYawPitch'], [q0], [T], True)
                # 放腿
                q = p2q([0, -0.02, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                q = p2q([0, 0.08, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                q = p2q([0, 0.05, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                q = p2q([0, 0.05, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                headlock()
                # 移重心
                q = p2q([0, 0.08, 0.28, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                q = p2q([0, -0.02, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                # 抬腿
                q = p2q([0, 0.10, 0.25, 0, 0, 0])
                q[0] = q0
                id = LLeg.set_joint([q], [T])
                q = p2q([0, -0.02, 0.28, 0, 0, 0], True)
                q[0] = q0
                id = RLeg.set_joint([q], [T])
                motion_proxy.wait(id, 1500)
                # 转腿
                motion_proxy.angleInterpolation(['LHipYawPitch'], [math.pi / 180 * 0], [T], True)
                # 放腿
                id = LLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0])], [T])
                id = RLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0], True)], [T])
                motion_proxy.wait(id, 1500)
                id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
                id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
                motion_proxy.wait(id, 1500)
                headlock()
            id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [T])
            id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [T])
            motion_proxy.wait(id, 1500)
            id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [T])
            id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [T])
            motion_proxy.wait(id, 1500)

    def kick(self):
        LLeg = self.LLeg
        RLeg = self.RLeg
        motion_proxy = self.motion_proxy
        LLeg.set_stiffness(1.0)
        RLeg.set_stiffness(1.0)
        id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [1.0])
        id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [1.0])
        motion_proxy.wait(id, 1500)
        # 移重心
        id = RLeg.set_joint([p2q([0, -0.02, 0.28, 0, 0, 0], True)], [1.0])
        id = LLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0])], [1.0])
        motion_proxy.wait(id, 1500)
        # 抬脚
        id = LLeg.set_joint([p2q([0, 0.09, 0.25, 0, 0, 0])], [1.0])
        motion_proxy.wait(id, 1500)
        # 前踢
        id = LLeg.set_joint([p2q([0.10, 0.09, 0.25, 0, 0, 0])], [0.2])
        motion_proxy.wait(id, 1500)
        # 收脚
        id = LLeg.set_joint([p2q([0, 0.08, 0.28, 0, 0, 0])], [1.0])
        motion_proxy.wait(id, 1500)
        # 恢复站立
        id = LLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0])], [2.0])
        id = RLeg.set_joint([p2q([0, 0.05, 0.28, 0, 0, 0], True)], [2.0])
        motion_proxy.wait(id, 1500)
        id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [1.0])
        id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [1.0])
        motion_proxy.wait(id, 1500)


if __name__ == '__main__':
    l_b1, l_b2, l_s, l_e = get_com_locus(math.pi/4)

    x_b1, y_b1, t_b1 = l_b1
    plt.plot(t_b1, x_b1, t_b1, y_b1)
    plt.show()
    plt.plot(x_b1, y_b1)
    plt.show()

    x_b2, y_b2, t_b2 = l_b2
    # plt.plot(t_b2, x_b2, t_b2, y_b2)
    # plt.show()
    # plt.plot(x_b2, y_b2)
    # plt.show()
    #
    x_s, y_s, t_s = l_s
    # plt.plot(t_s, x_s, t_s, y_s)
    # plt.show()
    # plt.plot(x_s, y_s)
    # plt.show()

    x_e, y_e, t_e = l_e
    plt.plot(t_e, x_e, t_e, y_e)
    plt.show()
    plt.plot(x_e, y_e)
    plt.show()

    x = [0]
    x_n = 0
    y = [0]
    y_n = 0
    t = [0]
    t_n = 0

    x = x + [x_n + x - x_s[0] for x in x_s]
    y = y + [y_n + y - y_s[0] for y in y_s]
    t = t + [t_n + t - t_s[0] for t in t_s]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b1[0] for x in x_b1]
    y = y + [y_n - y + y_b1[0] for y in y_b1]
    t = t + [t_n + t - t_b1[0] for t in t_b1]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b2[0] for x in x_b2]
    y = y + [y_n + y - y_b2[0] for y in y_b2]
    t = t + [t_n + t - t_b2[0] for t in t_b2]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b1[0] for x in x_b1]
    y = y + [y_n - y + y_b1[0] for y in y_b1]
    t = t + [t_n + t - t_b1[0] for t in t_b1]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b2[0] for x in x_b2]
    y = y + [y_n + y - y_b2[0] for y in y_b2]
    t = t + [t_n + t - t_b2[0] for t in t_b2]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b1[0] for x in x_b1]
    y = y + [y_n - y + y_b1[0] for y in y_b1]
    t = t + [t_n + t - t_b1[0] for t in t_b1]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_b2[0] for x in x_b2]
    y = y + [y_n + y - y_b2[0] for y in y_b2]
    t = t + [t_n + t - t_b2[0] for t in t_b2]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    x = x + [x_n + x - x_e[0] for x in x_e]
    y = y + [y_n - y + y_e[0] for y in y_e]
    t = t + [t_n + t - t_e[0] for t in t_e]
    x_n = x[-1]
    y_n = y[-1]
    t_n = t[-1]

    plt.plot(t, x, t, y)
    plt.show()
    plt.plot(x, y)
    plt.show()
    pass
