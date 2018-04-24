# encoding: utf-8
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from naoqi import ALProxy

import numpy as np
import time
import math

from naoobject import NAOJoint
from naoobject import NAOCamera
from naoobject import NAOPosture

from naowalk import Move
from naowalk import p2q

from user_locate import locate3d


head = None
cam = None
LLeg = None
RLeg = None
Body = None
posture = None


def headlock():
    """
    控制头部始终朝向乒乓球
    """
    global cam, head
    _, img = cam.get_frame()        # 获取下部摄像机图像
    yaw0, pitch0 = head.get_joint()     # 获取脖子角度
    x, y, yaw, pitch = locate3d(img, yaw0, pitch0)  # 从图像获取乒乓球位置
    if yaw is not None and pitch is not None:   # 角度范围检查
        if yaw > math.pi / 3:
            yaw = math.pi / 3
        if yaw < - math.pi / 3:
            yaw = -math.pi / 3
        if pitch<0.6928957:
            pitch = 0.6928957
        id = head.set_joint([[yaw, pitch - 0.6928957]], [0.8])  # 设定头部角度


def main():
    global cam, head, LLeg, RLeg, Body, posture

    ip = "169.254.81.2"
    motion_proxy = None
    try:
        motion_proxy = ALProxy("ALMotion", ip, 9559)
        cam_proxy = ALProxy("ALVideoDevice", ip, 9559)
        posture_proxy = ALProxy("ALRobotPosture", ip, 9559)
    except Exception:
        raise RuntimeError("Failed to create proxy.")

    # 需要控制的各种部分
    cam = NAOCamera(cam_proxy)
    LLeg = NAOJoint(motion_proxy, ['LLeg'])
    RLeg = NAOJoint(motion_proxy, ['RLeg'])
    LArm = NAOJoint(motion_proxy, ['LArm'])
    RArm = NAOJoint(motion_proxy, ['RArm'])
    Body = NAOJoint(motion_proxy, ["Body"])
    head = NAOJoint(motion_proxy, ["Head"])
    posture = NAOPosture(posture_proxy)

    # 初始位置：站立
    LLeg.set_stiffness(1.0)
    RLeg.set_stiffness(1.0)
    id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [1.0])
    id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [1.0])
    head.set_stiffness(1.0)
    id = head.set_joint([[0.0, 0.0]], [1.0])    # 头部初始位置
    motion_proxy.wait(id, 2000)

    # Move类，用于控制机器人移动
    move = Move(LLeg, RLeg, motion_proxy)   # 需要使用的关节
    _, img = cam.get_frame()    # 定位乒乓球，并把头对准乒乓球
    yaw0, pitch0 = head.get_joint()
    x, y, yaw, pitch = locate3d(img, yaw0, pitch0)
    if yaw is not None and pitch is not None:
        if yaw > math.pi/4:
            yaw = math.pi/4
        if yaw <- math.pi / 4:
            yaw = -math.pi / 4
        if pitch<0.6928957:
            pitch = 0.6928957
        id = head.set_joint([[yaw, pitch - 0.6928957]], [0.8])
        motion_proxy.wait(id, 2000)

    while x > 0.12 or x is None:    # 若距离乒乓球超过12cm，则继续移动
        _, img = cam.get_frame()
        yaw0, pitch0 = head.get_joint()
        x, y, yaw, pitch = locate3d(img, yaw0, pitch0)  # 定位乒乓球
        if yaw is not None and pitch is not None:   # 检查角度
            if yaw > math.pi / 4:
                yaw = math.pi / 4
            if yaw < - math.pi / 4:
                yaw = -math.pi / 4
            if pitch < 0.6928957:
                pitch = 0.6928957
            id = head.set_joint([[yaw, pitch - 0.6928957]], [0.8])  # 移动头部
            motion_proxy.wait(id, 2000)
            theta = math.atan(y*2/x)
            if abs(theta)>math.pi/180*10:
                move.turn(int(theta // (math.pi / 180 * 20)), headlock)     # 调整机器人朝向
                headlock()
            move.forward(x*2/3, math.atan(y/x), headlock)   # 向机器人移动

    _, img = cam.get_frame()
    yaw0, pitch0 = head.get_joint()
    x, y, yaw, pitch = locate3d(img, yaw0, pitch0)
    if yaw is not None and pitch is not None:
        if yaw > math.pi / 4:
            yaw = math.pi /4
        if yaw < - math.pi / 4:
            yaw = -math.pi / 4
        if pitch<0.6928957:
            pitch = 0.6928957
        id = head.set_joint([[yaw, pitch - 0.6928957]], [0.8])
        motion_proxy.wait(id, 2000)     # 最后定位乒乓球
        if y < 0:
            theta = math.atan(y / x)
            move.turn(int(theta // (math.pi / 180 * 20)), headlock)     # 调整机器人朝向
            headlock()
    move.kick()     # 踢球
    Body.set_stiffness(1.0)
    time.sleep(2)
    Body.rest()     # 机器人回到稳定的坐姿


if __name__ == '__main__':
    main()
