#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from offb.msg import kamera_verisi
from cv_bridge import CvBridge
import cv2
import rospy
import ros_numpy
import time
print(cv2.__version__)

"""
cv bridge python3 için uygun değil o yüzden rosrun değil terminalde python2 ile çalıştırıyorum
cv_bridge yerine numpy dizesi kullanılabilirmiş
"""


class RobotKamera():
    def __init__(self):
        global x, y, alan, yollanacak
        self.Conf_threshold = 0.40
        self.NMS_threshold = 0.60
        self.color = (255, 0, 0)
        self.box = []
        self.Label = "Target"
        self.h = 0.0
        self.w = 0.0
        self.isPlane = False
        self.tracked = False
        self.yollanacak = kamera_verisi()
        self.net = cv2.dnn.readNet("/home/burak/catkin_ws/src/offb/src/custom-yolov4-detector_best.weights", "/home/burak/catkin_ws/src/offb/src/custom-yolov4-detector.cfg")
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)

        rospy.init_node("kamera_dugumu", log_level=rospy.DEBUG)
        rospy.Subscriber("uav0/camera/image_raw", Image, self.kameraCallback)

        self.pub = rospy.Publisher('/kamera_verisi', kamera_verisi, queue_size=1)
        self.rate = rospy.Rate(15)
        self.bridge = CvBridge()
        rospy.spin()

    def yollama(self):
        self.yollanacak.header.stamp = rospy.Time.now()
        self.yollanacak.header.seq += 1
        self.yollanacak.header.frame_id = "kamera"
        self.yollanacak.x_yatay = self.h
        self.yollanacak.y_dikey = self.w
        self.yollanacak.isPlane = self.isPlane
        rospy.loginfo(self.yollanacak)
        self.pub.publish(self.yollanacak)

    def kameraCallback(self, mesaj):
        #self.foto = ros_numpy.numpify(mesaj)
        self.foto = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        if not self.tracked:
            classes, scores, boxes = self.model.detect(self.foto, self.Conf_threshold, self.NMS_threshold)
            for (classid, score, box) in zip(classes, scores, boxes):
                self.box = box
                self.tracked = True
                self.tracker = cv2.TrackerKCF_create()
                self.tracker.init(self.foto, self.box)
                self.isPlane=True
                self.yollama()
                cv2.rectangle(self.foto, self.box, self.color, 1)
                cv2.putText(self.foto, self.Label, (self.box[0], self.box[1] - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                            self.color, 2),
                break

        if self.tracked:
            ok, self.box = self.tracker.update(self.foto)
            if ok:
                print(self.box[0:2])
                self.h = (self.box[2]/2+self.box[0])
                self.w = (self.box[3]/2+self.box[1])
                cv2.rectangle(self.foto, self.box, self.color, 1)
                cv2.putText(self.foto, self.Label, (self.box[0], self.box[1] - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5, self.color, 2),
                self.isPlane=True

            else:
                self.tracked = False
                self.isPlane=False

        cv2.imshow("Robot Kamerasi", self.foto)
        cv2.waitKey(1)
        self.yollama()



nesne = RobotKamera()





