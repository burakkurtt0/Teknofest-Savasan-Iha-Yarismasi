#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
from offb.msg import kamera_verisi
from cv_bridge import CvBridge
import cv2
import rospy
import ros_numpy
import numpy as np
import time



class RobotKamera():
    def __init__(self):
        self.Conf_threshold = 0.25
        self.NMS_threshold = 0.25
        self.color = (255, 0, 0)
        self.box = []
        self.Label = "Target"
        self.h = 0.0
        self.w = 0.0
        self.isPlane = False
        self.tracked = False
        self.yollanacak = kamera_verisi()
        self.net = cv2.dnn.readNet("/home/burak/catkin_ws/src/offb/src/asd.weights",
                                   "/home/burak/catkin_ws/src/offb/src/custom-yolov4-detector.cfg")
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=False)

        rospy.init_node("kamera_dugumu")
        self.kamera_abone=rospy.Subscriber("uav0/camera/image_raw", Image, self.kameraCallback)
        #self.cap=cv2.VideoCapture(0)
        self.pub = rospy.Publisher('/kamera_verisi', kamera_verisi, queue_size=10)
        self.rate = rospy.Rate(30)
        rospy.spin()

        self.layer = self.net.getLayerNames()
        self.output_layer = [self.layer[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.color = (255, 0, 0)


    def yollama(self,h,w,isplane):
        self.yollanacak.header.stamp = rospy.Time.now()
        self.yollanacak.header.seq += 1
        self.yollanacak.header.frame_id = "kamera"
        self.yollanacak.x_yatay = h
        self.yollanacak.y_dikey = w
        self.yollanacak.isPlane = isplane
        rospy.loginfo(self.yollanacak)
        self.pub.publish(self.yollanacak)

    def kameraCallback(self, mesaj):
        bridge = CvBridge()

        #self.foto = ros_numpy.numpify(mesaj)
        self.foto = bridge.imgmsg_to_cv2(mesaj, "bgr8")
        cv2.waitKey(1)
        frame=cv2.resize(self.foto,(416,416))

        (H, W) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)

        self.net.setInput(blob)
        start_time = time.time()
        layerOutputs = self.net.forward(frame)
        end_time = time.time()
        boxes = []
        classIds = []
        confidences = []
        for output in layerOutputs:

            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)

                confidence = scores[classID]
                if confidence > self.Conf_threshold:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    classIds.append(classID)
                    confidences.append(float(confidence))

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.Conf_threshold, self.NMS_threshold)

        if len(idxs) > 0:
            #self.yollama(x, y, True)

            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
		self.yollama(x+(w/2), y+(h/2), True)

                cv2.rectangle(frame, (x, y), (x + w, y + h), self.color, 2)
                text = "Plane"
                cv2.putText(
                    frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
                )

                fps_label = "FPS: %.2f" % (1 / (end_time - start_time))
                cv2.putText(
                    frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2
                )

        else:
            self.yollama(0,0,False)
        cv2.imshow("Kuzgun İha", frame)
nesne = RobotKamera()
