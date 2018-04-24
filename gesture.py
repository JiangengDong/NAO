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


def analysis(pos):
    x, y, z = pos[0:3]
    theta = [0]*5
    theta[0] = math.atan2(-z, x) / math.pi * 180

    C1 = math.sin(theta[1]) * z - math.cos(theta[1]) * x
    C2 = -y
    temp1 = (C1 ** 2 + C2 ** 2 - 1677.69) / math.sqrt(2 * (44100 * C1 ** 2 + 900 * C2 ** 2))
    temp2 = math.atan2(30 * C1 - 210 * C2, 210 * C1 + 30 * C2)
    theta[1] = (math.asin(temp1) - temp2) / math.pi * 180 + 90  # this need +90 because difference in coordinate system

    theta[2] = 0

    sin24 = (C1 - 15 * math.cos(theta[2]) - 105 * math.sin(theta[2]))
    cos24 = -(C2 - 15 * math.sin(theta[2]) + 105 * math.cos(theta[2]))
    theta[3] = (math.atan2(sin24, cos24) - theta[2]) / math.pi * 180

    theta[4] = 0

    return [t*math.pi/180 for t in theta]


def main():
    ip = "169.254.81.2"
    motion_proxy = None
    try:
        motion_proxy = ALProxy("ALMotion", ip, 9559)
        cam_proxy = ALProxy("ALVideoDevice", ip, 9559)
        posture_proxy = ALProxy("ALRobotPosture", ip, 9559)
    except Exception:
        raise RuntimeError("Failed to create proxy.")

    head = NAOJoint(motion_proxy, ["Head"])
    cam = NAOCamera(cam_proxy)
    LLeg = NAOJoint(motion_proxy, ['LLeg'])
    RLeg = NAOJoint(motion_proxy, ['RLeg'])
    LArm = NAOJoint(motion_proxy, ["LArm"])
    RArm = NAOJoint(motion_proxy, ["RArm"])
    posture = NAOPosture(posture_proxy)


    posture.pose("Crouch")
    id = LLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0])], [1.0])
    id = RLeg.set_joint([p2q([0, 0.05, 0.30, 0, 0, 0], True)], [1.0])
    head.set_stiffness(1.0)
    motion_proxy.wait(id, 0)

    circle_pos = [[180, 20*math.cos(i/200), 20*math.sin(i/200)] for i in range(200)]
    Angle = [None]*200
    for i in range(200):
        Angle[i] = analysis(circle_pos[i])
    latency = 5
    t1 = [i/50.0+latency for i in range(200)]
    t2 = [(i+1) / 50 for i in range(200)]

    RArm.set_stiffness(1.0)
    id = RArm.set_joint(Angle, t1)
    motion_proxy.wait(id, 50000)
    posture.pose("Crouch")


if __name__ == '__main__':
    main()