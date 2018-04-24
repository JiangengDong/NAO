# encoding: utf-8
from __future__ import print_function

import cv2
import vision_definitions
import numpy as np


class NAOCamera(object):
    # Header for BMP picture
    BMPHeader = '\x42\x4d' \
                '\x36\x84\x03\x00' \
                '\x00\x00\x00\x00' \
                '\x36\x00\x00\x00' \
                '\x28\x00\x00\x00' \
                '\x40\x01\x00\x00' \
                '\xf0\x00\x00\x00' \
                '\x01\x00\x18\x00' \
                '\x00\x00\x00\x00\x00\x00\x00\x00' \
                '\x00\x00\x00\x00\x00\x00\x00\x00' \
                '\x00\x00\x00\x00\x00\x00\x00\x00'
    # parameter for camera
    resolution = vision_definitions.kQVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 40
    # number of saved pictures
    ImgNo = 0

    def __init__(self, proxy):
        self.__camProxy = proxy

        # name for two cameras
        self.__nameId = [None, None]
        self.subscribe()

    def __del__(self):
        self.un_subscribe()

    def subscribe(self):
        """
        subscribe to the two cameras of NAO

        :return: True if success
        """
        self.__nameId[0] = self.__camProxy.subscribeCamera('Camera_0', 0,
                                                           NAOCamera.resolution,
                                                           NAOCamera.colorSpace,
                                                           NAOCamera.fps)
        self.__nameId[1] = self.__camProxy.subscribeCamera('Camera_1', 1,
                                                           NAOCamera.resolution,
                                                           NAOCamera.colorSpace,
                                                           NAOCamera.fps)
        if self.__nameId[0] is not None and self.__nameId[1] is not None:
            return True
        else:
            return False

    def un_subscribe(self):
        """
        unsubscribe to the twos cameras

        :return: True if success
        """
        if self.__nameId[0] is None and self.__nameId[1] is None:
            pass
        else:
            self.__camProxy.unsubscribe(self.__nameId[0])
            self.__camProxy.unsubscribe(self.__nameId[1])
            self.__nameId = [None, None]
        return True

    def get_frame(self):
        """
        receive data packages from NAO and transform them into frames

        :return: {frame of camera 0, frame of camera 1}
        """
        img = [None, None]

        # get data packages of two cameras from NAO
        img[0] = self.__camProxy.getImageRemote(self.__nameId[0])
        self.__camProxy.releaseImage(self.__nameId[0])
        img[1] = self.__camProxy.getImageRemote(self.__nameId[1])
        self.__camProxy.releaseImage(self.__nameId[1])

        # process the data packages and get frames
        if img[0] is not None:
            # decode
            img[0] = np.fromstring(NAOCamera.BMPHeader + img[0][6], dtype=np.uint8)
            img[0] = cv2.imdecode(img[0], 1)
            img[0] = cv2.flip(img[0], 0)
        if img[1] is not None:
            # decode
            img[1] = np.fromstring(NAOCamera.BMPHeader + img[1][6], dtype=np.uint8)
            img[1] = cv2.imdecode(img[1], 1)
            img[1] = cv2.flip(img[1], 0)
        return img

    def run(self):
        """
        run the client and display the video

        :return: None
        """
        cv2.namedWindow('Camera_0', 1)
        cv2.namedWindow('Camera_1', 1)

        try:
            while True:
                try:
                    frame = self.get_frame()

                    # display
                    cv2.imshow('Camera_0', frame[0])
                    cv2.imshow('Camera_1', frame[1])
                except KeyboardInterrupt:
                    break
                key = cv2.waitKey(10)
                if key == 27:
                    break
                elif key == ord('s'):
                    cv2.imwrite('No%d_Camera0.bmp' % NAOCamera.ImgNo, frame[0])
                    cv2.imwrite('No%d_Camera1.bmp' % NAOCamera.ImgNo, frame[1])
                    NAOCamera.ImgNo += 1
        finally:
            cv2.destroyAllWindows()


class NAOJoint(object):
    LINK_LIST = {"LArm": ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                 "RArm": ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
                 "LLeg": ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                 "RLeg": ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                 "Head": ['HeadYaw', 'HeadPitch'],
                 "Body": []}

    def __init__(self, proxy, link_list):
        self.NameList = []
        self.proxy = proxy
        self.Link = []
        # 根据需要控制的连杆，存储需要控制的关节
        for link in link_list:
            if link in NAOJoint.LINK_LIST.keys():
                self.Link = self.Link + [link]
                self.NameList = self.NameList + NAOJoint.LINK_LIST[link]
            else:
                raise RuntimeError('Invalid link.')

    def get_stiffness(self):
        return self.proxy.getStiffnesses(self.Link)

    def set_stiffness(self, value):
        self.proxy.setStiffnesses(self.Link, value)

    def rest(self):
        self.proxy.rest()

    def __del__(self):
        self.set_stiffness(0.0)

    def set_joint(self, angle_list, time_list):
        """
        Set angles of motors on a link.
        This method is realized with a 'post' method, which is not blocking.

        :param angle_list: [[angles at t0], [angles at t1], ..., [angles at tn]]
        :param time_list: [t0, t1, ..., tn]
        :return: id: thread id for the 'post' method
        """
        if len(angle_list[0]) == len(self.NameList) and len(angle_list) == len(time_list):
            angle_list = [[row[col] for row in angle_list] for col in range(len(angle_list[0]))]
            time_list = [time_list] * len(angle_list)
            id = self.proxy.post.angleInterpolation(self.NameList, angle_list, time_list, True)
        else:
            raise RuntimeError('Length of parameters does not correspond to that of links.')
        return id

    def get_joint(self):
        return self.proxy.getAngles(self.NameList, True)


class NAOPosture(object):
    POSE_LIST = ["Stand", "StandInit", "StandZero", "Crouch", "Sit", "SitRelax", "LyingBelly", "LyingBack"]

    def __init__(self, proxy):
        self.postureProxy = proxy

    def pose(self, name):
        if name in NAOPosture.POSE_LIST:
            if self.postureProxy is not None:
                result = self.postureProxy.goToPosture(name, 0.8)
                if result:
                    print(name, ' Success.')
                else:
                    raise RuntimeError(
                        "%s is not a part of the standard posture library or robot cannot reach the posture" % name)
        else:
            raise RuntimeError("Invalid posture.")
