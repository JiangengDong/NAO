# encoding: utf-8


### 识别类型用原色，调节用手套（左手）

import cv2
import numpy as np
import math
import time

from naoqi import ALProxy
from naoobject import NAOCamera
from naoobject import NAOJoint
from naowalk import Move


# 找到颜色，输入一张只含待辨识颜色的照片
def findcolor(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    shape =  img.shape
    for i in range(3):
        new_min = []
        new_max = []
        for j in range(shape[0]):
            new_min.append(min(img[j,:,i]))
            new_max.append(max(img[j,:,i]))
        print min(new_min), max(new_max)


# 平均背景法，读取40张图片中的背景，求取图片像素的平均值和绝对差
# 阈值设为[mean-constant*diff，mean+constant*diff]，在这中间的为背景
def background(n):
    mean = 0
    thre = 25 #系数可调
    for i in range(n):
        img = cam.get_frame()
        img = img[0]
        # cv2.imwrite("photo%d.png"%(i+1),img)
        mean_per = i/(i+1)
        mean = cv2.addWeighted(mean,mean_per,img,1-mean_per,0)
        if i > 0:
            diff_per = (i-1)/i
            diff = cv2.absdiff(img,pre_img)
            mean_diff = cv2.addWeighted(diff,diff_per,diff,1-diff_per,0)
        pre_img = img
        time.sleep(0.1)

    back_low = mean - thre*mean_diff
    back_high = mean + thre*mean_diff
    return back_low,back_high


# 针对肤色的thresh
def thresh(img):
    upper_color = np.array([25, 145, 255])
    lower_color = np.array([3, 35, 30])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


# 针对彩色手套的thresh
def thresh2(img):
    upper_color = np.array([25, 145, 255])
    lower_color = np.array([5, 35, 30])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


# 找到掩模图像中的形状轮廓
def handcontour(img):
    mask = thresh2(img)
    # cv2.imshow('hand', maska)
    median = cv2.medianBlur(mask, 5)
    median = cv2.erode(median, None, iterations=2)
    mask = cv2.dilate(median, None, iterations=2)
    mask = cv2.medianBlur(mask, 5)

    #找到轮廓
    image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    max = 0
    maxarea = 0
    #如果没有轮廓则识别失败返回None
    if len(contours) == 0:
        return []
    #找到面积最大的轮廓为手
    for i in range(len(contours)):
        if cv2.contourArea(contours[i]) > maxarea:
            maxarea = cv2.contourArea(contours[i])
            max = i
    img = cv2.drawContours(img, contours, max, (0, 0, 255), 2)
    # cv2.imshow('hand',img)
    cv2.waitKey(0)
    return contours[max] #返回手的轮廓


# 通过模板匹配识别手的姿势
def gesture(shape,kinds):
    best = 1
    result = 0
    for i in range(kinds):
        tempshape = np.load('.\data\gg%d.npy' % i)
        ret = cv2.matchShapes(shape, tempshape, 1, 0.0)
        if ret < best:
            best = ret
            result = i + 1
    return ret, result


# 找到手的轮廓的外接矩形，返回矩形中心，没找到就返回None
def handloc(contour):
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx,cy


# 根据机器人读取的图像识别动作
def trace(cam):  # 移动方向
    x = []
    y = []
    n = 7 # 连续获取n帧图像
    for i in range(n):
        # 此处应是获取一幅图，然后等待一段时间
        time.sleep(0.2)
        list_img,_ = cam.get_frame()
        list_contour = handcontour(list_img)    # 两个back一样 相当于没有背景
        # cv2.imshow("window",list_img)
        # cv2.waitKey(0)
        while len(list_contour)<50:
            img = cam.get_frame()
            list_img = img[0]
            list_contour = handcontour(list_img)
        print len(list_contour)
        x0,y0 = handloc(list_contour)
        if x0 != 0 and y0 != 0:
            x.append(x0)
            y.append(y0)

    x = np.array(x)
    y = np.array(y)
    print x,y
    cv2.imshow('frame', list_img)
    cv2.waitKey(500)
    cv2.destroyWindow('frame')
    grad_x = []
    grad_y = []
    for i in range(n-1):
        grad_x0 = x[i+1] - x[i]
        grad_y0 = y[i+1] - y[i]
        mod = math.sqrt(grad_x0 ** 2 + grad_y0 ** 2)
        if mod != 0:
            grad_x0 = grad_x0/mod
            grad_y0 = grad_y0/mod
            grad_x.append(grad_x0)
            grad_y.append(grad_y0)
    grad_x = np.array(grad_x)
    grad_y = np.array(grad_y)
    abs_x = abs(max(x)-min(x))
    abs_y = abs(max(y)-min(y))
    return grad_x,grad_y,abs_x,abs_y


# hausdorff距离函数
def hausdist(grad_x,grad_y,temp_x,temp_y):
    max_a = 0
    max_b = 0
    dist = 0
    min_t = 500
    for i in range(grad_x.shape[0]):
        for j in range(temp_x.shape[0]):
            dist = math.sqrt((grad_x[i]-temp_x[j])**2 + (grad_y[i]-temp_y[j])**2)
            if dist < min_t:
                min_t = dist
        if min_t > max_a:
            max_a = min_t
    for i in range(temp_x.shape[0]):
        for j in range(grad_x.shape[0]):
            dist = math.sqrt((temp_x[i]-grad_x[j])**2 + (temp_y[i]-grad_y[j])**2)
            if dist < min_t:
                min_t = dist
        if min_t > max_b:
            max_b = min_t
    return max(max_a,max_b)


# 识别手部动作，在识别出触发左手或右手动作后的姿势后调用
def handaction(cam):
    mm = {1:"向下",2:"向上",3:"向左",4:"向右",5:"顺时针",6:"逆时针"}
    mode = 0
    haus = 500
    dist = 500
    grad_x,grad_y,abs_x,abs_y = trace(cam)
    print abs_x,abs_y
    if True: # 圆，但是要判断正逆时针
        tx1 = grad_x[0] + grad_x[1]
        ty1 = grad_y[0] + grad_y[1]
        tx2 = tx1 / 2 - grad_x[0]
        ty2 = ty1 / 2 - grad_y[0]
        direction = tx1 * ty2 - ty1 * tx2
        if direction > 0:  # 拟时针
            mode = 6  # 顺时针代号为5
        else:
            mode = 5
    print "mode ",mode, mm[mode]
    return mode


# 腿部动作识别
def legaction(cam):
    mm = {0: "backward", 1: "forward", 2: "left", 3: "right"}
    contourarea = []
    # 这个10根据图片数量可以改
    n = 10
    x = []
    y = []
    for i in range(n):
        time.sleep(0.2)
        img = cam.get_frame()
        img = img[0]
        contour = handcontour(img)
        while len(contour)<50:
            print len(contour)
            img = cam.get_frame()
            img = img[0]
            contour = handcontour(img)
        x0, y0 = handloc(contour)
        x.append(x0)
        y.append(y0)
        contourarea.append(cv2.contourArea(contour))
    finalmove = contourarea[n - 1] - contourarea[0]
    for i in range(n - 1):
        contourarea[i] = contourarea[i + 1] - contourarea[i]
        if contourarea[i] < 0:
            contourarea[i] = -1
        else:
            contourarea[i] = 1
    contourarea[n - 1] = 0
    contourarea = np.array(contourarea, dtype='int')
    print contourarea, finalmove
    print abs(x[n - 1] - x[0])
    if abs(x[n - 1] - x[0]) < 50:
        if np.sum(contourarea) > round(0.3 * n) and finalmove > 0:
            result = 1  # 向前是1
        else:
            # elif np.sum(contourarea) < -round(0.3*n) and finalmove < 0:
            result = 0  # 向后是0
    else:
        x = np.array(x)
        y = np.array(y)
        grad_x = []
        grad_y = []
        for i in range(n - 1):
            grad_x0 = x[i + 1] - x[i]
            grad_y0 = y[i + 1] - y[i]
            mod = math.sqrt(grad_x0 ** 2 + grad_y0 ** 2)
            grad_x0 = grad_x0 / mod
            grad_y0 = grad_y0 / mod
            grad_x.append(grad_x0)
            grad_y.append(grad_y0)
        grad_x = np.array(grad_x)
        grad_y = np.array(grad_y)
        # grad_x, grad_y = trace()
        haus = 500
        for i in range(2, 4):  # 只有左右移动
            temp_x = np.load('.\data\motion%d.npy' % i)
            temp_y = np.load('.\data\motion%d.npy' % i)
            dist = hausdist(grad_x, grad_y, temp_x, temp_y)
            if dist < haus:
                haus = dist
                result = i  # 向左是2 向右是3这样
    print result,mm[result]
    return result


class HeadFollow(object):
    def __init__(self, cam, head):
        self.i = 0
        self.errorx_pre = 0
        self.errory_pre = 0
        self.errorlist_x = []
        self.errorlist_y = []
        self.temp_con = np.zeros([2, 3])
        self.cam = cam
        self.head = head

    # 这个函数是实时的，需要循环调用,在调用之前应当先记录一些图片以识别背景
    def follow(self):
        img, _ = cam.get_frame()
        contour = handcontour(img)
        while len(contour)<50:
            img, _ = cam.get_frame()
            contour = handcontour(img)
        x, y = handloc(contour)
        print(x, y)
        yaw0, pitch0 = head.get_joint()
        pitch = (math.atan((y - 120) / 271.8194)) + pitch0
        yaw = math.atan((160 - x) / 271.7887) + yaw0
        id = head.set_joint([[yaw, pitch]], [0.3])
        motion_proxy.wait(id, 2000)


def get_gesture():
    # ges = {"1":"拳头","2":"一根手指","3":"Yeah","4":"手掌","5":"六","6":"四根手指","7":"八"}
    ges = {"1": "yeah", "2":"1", "3":"8"}
    kinds = len(ges)

    # 从摄像头获取一张图片

    # val = input("Enter 1 to continue:") # 对应之后机器人拍脑袋或者怎么样接收动作
    ret = 1
    img, _ = cam.get_frame()
    contour = handcontour(img)
    while ret > 0.55 or len(contour) < 30 :
        print "Matching ..."
        print len(contour)
        img, _ = cam.get_frame()
        contour = handcontour(img)
        if len(contour) > 0:
            ret, g = gesture(contour,kinds)
        time.sleep(0.2)

    cx, cy = handloc(contour)
    print "Match degree %f !(0 is perfect)" % ret
    print "Gesture %d ("%g, ges['%d'%g],") matched!"   # 对应机器人语音提示
    print "Center = ", cx,cy
    cv2.imshow("gesture",img)
    cv2.waitKey(2000)
    cv2.destroyWindow("gesture")

    return g


if __name__ == "__main__":
    ip = "169.254.81.2"
    cam_proxy = None
    tts = None
    try:
        cam_proxy = ALProxy("ALVideoDevice", ip, 9559)
        tts = ALProxy("ALTextToSpeech", ip, 9559)
        motion_proxy = ALProxy("ALMotion", ip, 9559)
    except Exception:
        raise RuntimeError("Failed to create proxy.")

    cam = NAOCamera(cam_proxy)
    head = NAOJoint(motion_proxy, ["Head"])
    LLeg = NAOJoint(motion_proxy, ["LLeg"])
    RLeg = NAOJoint(motion_proxy, ["RLeg"])

    head.set_stiffness(1.0)
    head.set_joint([[0.0, 0.0]], [1.0])
    ges = {"1": "yeah", "2":"6", "3":"8"}

    for i in range(2):
        time.sleep(1)
        img =cam.get_frame()
        img = img[0]
        tts.say("开始识别。")
        time.sleep(2)

        head_follow = HeadFollow(cam, head)

        g = get_gesture()
        # tts.say("%d"%g)
        # tts.say("Done. Done.")

        time.sleep(3)
        if g == 1:
            exit()
            tts.say("手势夜。开始跟踪手部。")
            head.set_stiffness(1.0)
            for i in range(40):
                head_follow.follow()
            head.set_joint([[0.0, 0.0]], [1.0])

        elif g == 2:  # 手掌  -- 控制手臂
            tts.say("手势1。")
            mode = handaction(cam)  # back_low 不重要，只是为了凑个参数
            tts.say("模式%d。"%mode)
            RArm = NAOJoint(motion_proxy, ["RArm"])
            RArm.set_stiffness(1.0)
            RArm.set_joint([[math.pi/2, -math.pi/6, math.pi/2, math.pi/3, 0]], [1.0])
            if mode >= 5:      # 六种mode，对应向上、下、左、右、顺、逆时针转动
                l = 0.2  # 检测间隙0.2s
                a = 0  # 现在的角度（通过函数获取）
                # 设定电机转动
                time.sleep(1)
                img2, _ = cam.get_frame()
                contour2 = handcontour(img2)
                x_now, y_now = handloc(contour2)
                while True:  # 现在的角度没有到达极限（留一点余地
                    time.sleep(l)
                    x_pre = x_now
                    y_pre = y_now
                    img2, _ = cam.get_frame()
                    contour2 = handcontour(img2)
                    if len(contour2) > 0:
                        x_now, y_now = handloc(contour2)
                    dis = math.sqrt(abs(x_pre - x_now) ** 2 + abs(y_pre - y_now) ** 2)
                    if dis <= 10:
                        break
                    if mode == 5: a = a + 3
                    if mode == 6: a = a - 3
                    if a > 115:
                        a = 115
                    if a < -115:
                        a = -115
                    RArm.set_joint([[math.pi / 2, -math.pi / 6, math.pi / 2, math.pi / 3, a*math.pi/180]], [1.0])
            RArm.rest()

        elif g == 3:  # yeah  -- 控制腿
            tts.say("手势8。")
            mode = legaction(cam)   # back_low 不重要，只是为了凑个参数
            tts.say("模式%d"%mode)    # 四种mode，对应左转、右转、前进、后退
            move = Move(LLeg, RLeg, motion_proxy)
            q = 0
            if mode is 1:
                q = math.pi
            if mode is 0:
                q = 0
            if mode is 3:
                q = -math.pi/2
            if mode is 2:
                q = math.pi/2
            def nop(): pass
            move.forward(0.16, q, nop)
            Body = NAOJoint(motion_proxy, ["Body"])
            Body.set_stiffness(1.0)
            time.sleep(1)
            Body.rest()
            time.sleep(5)


    cv2.destroyAllWindows()
