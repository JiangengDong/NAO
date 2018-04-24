# -*- encoding: UTF-8 -*-

import sys
import time
from naoqi import ALProxy
import math
import almath

class InvKine:
    @staticmethod
    def analysis(pos):
        x, y, z = pos[0:3]
        theta = []
        theta[1] = math.atan2(-z, x)/math.pi*180

        C1 = math.sin(theta[1])*z-math.cos(theta[1])*x
        C2 = -y 
        temp1 = (C1**2+C2**2-1677.69)/math.sqrt(2*(44100*C1**2+900*C2**2))
        temp2 = math.atan2(30*C1-210*C2, 210*C1+30*C2)
        theta[2] = (math.asin(temp1) - temp2)/math.pi*180+90 # this need +90 because difference in coordinate system

        theta[3] = 0

        sin24 = (C1-15*math.cos(theta[2])-105*math.sin(theta[2]))
        cos24 = -(C2-15*math.sin(theta[2])+105*math.cos(theta[2]))
        theta[4] = (math.atan2(sin24, cos24) - theta[2])/math.pi*180

        theta[5] = 0

        return theta


def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("LArm", 1.0)
    motionProxy.setStiffnesses("RArm", 1.0)

    # Example showing multiple trajectories
    LNames = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    RNames = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    LPos = [[200, 0, 0], [200, 50, 0], [200, 50, -50], [200, 0, -50], [200, 0, 0]]
    LAngle = [[], [], [], [], []]
    for i in range(0, 5):
        tempAngle = InvKine.analysis(LPos[i])
        for j in range(0, 5):
            LAngle[j][i] = tempAngle[j]
    timeTemp = [1.0, 2.0, 3.0, 4.0, 5.0]

    names  = LNames
    angleLists = LAngle
    timeLists = [timeTemp, timeTemp, timeTemp, timeTemp, timeTemp]
    isAbsolute  = True

    motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    motionProxy.setStiffnesses("LArm", 0.0)
    motionProxy.setStiffnesses("RArm", 0.0)


if __name__ == "__main__":
    robotIp = "169.254.81.2"

    if len(sys.argv) <= 1:
        print "Usage python almotion_angleinterpolation.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)