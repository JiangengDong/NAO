from naoqi import ALProxy

from naoobject import NAOCamera
from naoobject import NAOPosture
from naoobject import NAOPosture
import locate_user
import cv2
import time
import math


def removed(ip, port, names='Body'):
    try:
        motionProxy = ALProxy("ALMotion", ip, port)
        motionProxy.setStiffnesses(names, 0)
        print("Stiffness removed.")
    except:
        print("Could not create proxy to ALMotion")


def handsdown(ip, port):
    try:
        motionProxy = ALProxy("ALMotion", ip, port)
    except:
        print("Could not create proxy to ALMotion")

    namelist = ['LShoulderPitch', 'RShoulderPitch']
    anglelist = [math.pi/2, math.pi/2]
    timelist = [1.0, 1.0]

    motionProxy.angleInterpolation(namelist, anglelist, timelist, True)


if __name__ == "__main__":
    ip = "169.254.81.2"
    port = 9559
    # walk = NAOWalk(ip, port)
    # walk.walkTo(1, 0, 0)
    posture = NAOPosture(ip, port)

    cameraClient = NAOCamera(ip, port)
    cameraClient.subscribe()

    walk = NAOWalk(ip, port)

    memoryProxy = ALProxy("ALMemory", ip, port)

    try:
        theta = None
        while True:
            posture.pose()
            handsdown(ip, port)
            time.sleep(0.5)
            img = cameraClient.get_frame()
            cv2.imshow('', img[1])
            cv2.imwrite('0.bmp', img[0])
            cv2.imwrite('1.bmp', img[1])
            cv2.waitKey(500)
            x, y = locate_user.locate3d(img[0], img[1])
            if x is None or y is None:
                walk.walkTo(0.3, 0.03, theta)
                break
            if x < 500:
                walk.walkTo(x/1000, y/1000, math.atan(y/x)*1.2)
                break
            x = x / 1000
            y = y / 1000
            if y>0:
                theta = math.atan(y/x)*1.3
            else:
                theta = math.atan(y / x) * 1.5
            walk.walkTo(x / 3, y / 3, theta)
        # while True:
        #     LFBumperL = memoryProxy.getData("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value")
        #     LFBumperR = memoryProxy.getData("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value")
        #     RFBumperL = memoryProxy.getData("Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value")
        #     RFBumperR = memoryProxy.getData("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value")
        #     break
    finally:
        cameraClient.un_subscribe()
        posture.pose("Crouch")
        removed(ip, port)