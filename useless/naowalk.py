from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import math
import numpy as np
import matplotlib.pyplot as plt


N = 150


def get_foot_locus(dx):
    s_x = 0.04
    s_y = 0.10

    n = int(dx // s_x) + 1
    x = [0.0] + [i * s_x for i in range(n)] + [dx, dx]
    y = [(-1) ** (i + 1) * s_y / 2 for i in range(n + 3)]
    pos_r = [None] * (2 * (n + 2))
    pos_l = [None] * (2 * (n + 2))

    for i in range(n + 2):
        if i % 2 == 0:
            pos_r[2 * i] = [x[i], y[i], 0]
            pos_l[2 * i] = [x[i], y[i] + 0.10, 0.01]
            pos_r[2 * i + 1] = [x[i], y[i], 0]
            pos_l[2 * i + 1] = [x[i + 1], y[i + 1], 0]

        if i % 2 == 1:
            pos_l[2 * i] = [x[i], y[i], 0]
            pos_r[2 * i] = [x[i], y[i] - 0.10, 0.01]
            pos_l[2 * i + 1] = [x[i], y[i], 0]
            pos_r[2 * i + 1] = [x[i + 1], y[i + 1], 0]

    return pos_l, pos_r


def get_com_locus():
    s_x = 0.04
    s_y = 0.12
    z = 0.3
    g = 9.8
    T_c = math.sqrt(z / g)
    T = 0.6
    vx_t = s_x / 2 * (math.cosh(T / T_c) + 1) / (T_c * math.sinh(T / T_c))
    vy_t = s_y / 2 * (math.cosh(T / T_c) - 1) / (T_c * math.sinh(T / T_c))
    vx_0 = -s_x / 2 / T_c * math.sinh(T / 2 / T_c) + vx_t * math.cosh(T / 2 / T_c)

    def xfun(t):
        return -s_x / 2 * math.cosh(t / T_c) + T_c * vx_t * math.sinh(t / T_c)

    def yfun(t):
        return s_y / 2 * math.cosh(t / T_c) + T_c * (-vy_t) * math.sinh(t / T_c)

    # locus for each step, except the first two
    comx_between = [0.0] * N
    comy_between = [0.0] * N
    t_between = [0.0] * N
    for i in range(N):
        t_between[i] = (i+1) * T / N
        comx_between[i] = xfun(t_between[i])
        comy_between[i] = yfun(t_between[i])
    locus_between = (comx_between, comy_between, t_between)

    # locus for the first step
    comx_s1 = [0.0] * N
    comy_s1 = [0.0] * N
    t_s1 = [0.0] * N

    def yfun2(t):
        return vy_t/2/T**3*t**4-vy_t/2/T*t**2+0.05

    for i in range(N):
        t_s1[i] = (i+1)*T/N
        comy_s1[i] = yfun2(t_s1[i])
        comx_s1[i] = s_x/2/math.cosh(T/T_c)*(t_s1[i]/T)**2
    locus_s1 = (comx_s1, comy_s1, t_s1)

    # locus for the second step
    comx_s2 = [0.0] * N
    comy_s2 = [0.0] * N
    t_s2 = [0.0] * N

    def xfun(t):
        return s_x/2/math.cosh(T/T_c)*math.cosh(t/T_c)

    for i in range(N):
        t_s2[i] = (i+1) * T / N
        comy_s2[i] = yfun(t_s2[i])
        comx_s2[i] = xfun(t_s2[i])
    locus_s2 = (comx_s2, comy_s2, t_s2)

    # locus for the second last step
    comx_e2 = [-x for x in comx_s2][-1::-1]
    locus_e2 = (comx_e2, comy_s2, t_s2)

    # locus for the last step
    comx_e1 = [0]*N
    comy_e1 = comy_s1[-1::-1]
    locus_e1 = (comx_e1, comy_e1, t_s1)

    return locus_between, locus_s1, locus_s2, locus_e1, locus_e2
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


def p2q(p, is_left=False):  # right leg
    q = [0] * 6
    x, y, z, _, _, _ = p
    y = y - 0.05
    z = z - 0.04519 - 0.085
    r = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    l1 = 0.1
    l2 = 0.1029
    q[1] = math.atan(y / z)
    q[5] = -q[1]
    q[3] = math.pi - math.acos((l1 ** 2 + l2 ** 2 - r ** 2) / 2 / l1 / l2)
    q[2] = -math.asin(x / r) - math.acos((l1 ** 2 + r ** 2 - l2 ** 2) / 2 / r / l1)
    q[4] = -q[3] - q[2]
    if is_left is True:
        q[1] = -q[1]
        q[5] = -q[5]
    return q


if __name__ == '__main__':
    l_b, l_s1, l_s2, l_e1, l_e2 = get_com_locus()

    x_s2, y_s2, t_s2 = l_s2
    q_l = [None]*N
    q_r = [None]*N
    for i in range(N):
        q_r[i] = p2q([0-x_s2[i], y_s2[i], 0.3, 0, 0, 0], True)
        q_l[i] = p2q([0-x_s2[i], y_s2[i], 0.3, 0, 0, 0])
    np.savez("Locus/Locus_s2.npz", q_l=q_l, t=t_s2, q_r=q_r)
    plt.plot(t_s2, x_s2, t_s2, y_s2)
    plt.show()

    x_s1, y_s1, t_s1 = l_s1
    for i in range(N):
        q_r[i] = p2q([0-x_s1[i], y_s1[i], 0.3, 0, 0, 0], True)
        q_l[i] = p2q([0 - x_s1[i], y_s1[i], 0.3, 0, 0, 0])
    np.savez("Locus/Locus_s1.npz", q_l=q_l, t=t_s1, q_r=q_r)
    plt.plot(t_s1, x_s1, t_s1, y_s1)
    plt.show()

    x_b, y_b, t_b = l_b
    for i in range(N):
        q_r[i] = p2q([0-x_b[i], y_b[i], 0.3, 0, 0, 0], True)
        q_l[i] = p2q([0 - x_b[i], y_b[i], 0.3, 0, 0, 0])
    np.savez("Locus/Locus_b.npz", q_l=q_l, t=t_b, q_r=q_r)
    plt.plot(t_b, x_b, t_b, y_b)
    plt.show()

    x_e1, y_e1, t_e1 = l_e1
    for i in range(N):
        q_r[i] = p2q([0 - x_e1[i], y_e1[i], 0.3, 0, 0, 0], True)
        q_l[i] = p2q([0 - x_e1[i], y_e1[i], 0.3, 0, 0, 0])
    np.savez("Locus/Locus_e1.npz", q_l=q_l, t=t_e1, q_r=q_r)
    plt.plot(t_e1, x_e1, t_e1, y_e1)
    plt.show()

    x_e2, y_e2, t_e2 = l_e2
    for i in range(N):
        q_r[i] = p2q([0 - x_e2[i], y_e2[i], 0.3, 0, 0, 0], True)
        q_l[i] = p2q([0 - x_e2[i], y_e2[i], 0.3, 0, 0, 0])
    np.savez("Locus/Locus_e2.npz", q_l=q_l, t=t_e2, q_r=q_r)
    plt.plot(t_e2, x_e2, t_e2, y_e2)
    plt.show()

    x = [0]
    x_n = 0
    y = [0]
    y_n = 0

    x = x + [x_n + x - x_s1[0] for x in x_s1]
    y = y + [y_n + y - y_s1[0] for y in y_s1]
    x_n = x[-1]
    y_n = y[-1]

    x = x + [x_n + x - x_s2[0] for x in x_s2]
    y = y + [y_n - y + y_s2[0] for y in y_s2]
    x_n = x[-1]
    y_n = y[-1]

    x = x + [x_n + x - x_b[0] for x in x_b]
    y = y + [y_n + y - y_b[0] for y in y_b]
    x_n = x[-1]
    y_n = y[-1]

    x = x + [x_n + x - x_b[0] for x in x_b]
    y = y + [y_n - y + y_b[0] for y in y_b]
    x_n = x[-1]
    y_n = y[-1]

    x = x + [x_n + x - x_e2[0] for x in x_e2]
    y = y + [y_n + y - y_e2[0] for y in y_e2]
    x_n = x[-1]
    y_n = y[-1]

    x = x + [x_n + x - x_e1[0] for x in x_e1]
    y = y + [y_n - y + y_e1[0] for y in y_e1]
    x_n = x[-1]
    y_n = y[-1]

    plt.plot(range(len(x)), x, range(len(y)), y)
    plt.show()
    pass
