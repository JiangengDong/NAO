from __future__ import print_function
from naoqi import ALProxy

import cv2
import vision_definitions
import struct

import sys
import time
import math
import numpy as np

ip = "169.254.81.2"
port = 9559

try:
    camProxy = ALProxy("ALVideoDevice", ip, port)
except Exception as e:
    print("Could not create proxy to ALVideoDevice")
    print("Error was: ", e)
    sys.exit(1)

resolution = vision_definitions.kQVGA
colorSpace = vision_definitions.kBGRColorSpace
fps = 30

nameId = "Camera_0"
camProxy.unsubscribe(nameId)
nameId = "Camera_1"
camProxy.unsubscribe(nameId)