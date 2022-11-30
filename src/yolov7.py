import torch
import cv2 as cv
from sensor_msgs.msg import Image
from offb.msg import kamera_verisi
from cv_bridge import CvBridge
import rospy
import ros_numpy
import numpy as np
import time

from header import yolov5_path, model_source, searching_for_uav, chasing_uav, uav_detected, kirmizi, mavi, yesil, beyaz, siyah, NMS_threshold, Conf_threshold, box, label, isPlane, tracked

class RobotKamera():
    def __init__(self):
        self.x=0
        self.y=0
        #yolo modelinin load kÄ±smÄ±
        self.model = torch.hub.load(yolov5_path,
                               'custom',
                               model_source,
                               source='local')
        self.model.cuda(0)

        #rospy baÄŸlantÄ± kodlarÄ±
        rospy.init_node("kamera_dugumu")
        self.kamera_abone=rospy.Subscriber("uav0/camera/image_raw", Image, self.kameraCallback)
        self.pub = rospy.Publisher('/kamera_verisi', kamera_verisi, queue_size=1)
        self.yollanacak = kamera_verisi()
        self.rate = rospy.Rate(30)

        rospy.spin()

    def yollama(self,h,w,isplane):
        self.yollanacak.header.stamp = rospy.Time.now()
        self.yollanacak.header.seq += 1
        self.yollanacak.header.frame_id = "kamera"
        self.yollanacak.x_yatay = h #burasÄ±nÄ± deÄŸiÅŸtir en son
        self.yollanacak.y_dikey = w
        self.yollanacak.isPlane = isplane
        rospy.loginfo(self.yollanacak)
        self.pub.publish(self.yollanacak)


    def kameraCallback(self, mesaj):
        frame = ros_numpy.numpify(mesaj)
        start_time=time.time()
        result = self.model(frame)
        end_time= time.time()
        ucaklar = result.pandas().xywh[0]
        print("***********")
        print(ucaklar)
        if ucaklar.__len__() > 0:
            a = ucaklar[0:1]
            if (float(a['confidence'][0:1]) > Conf_threshold):  # EÄŸer uÃ§ak olduÄŸuna yÃ¼zde x eminse
                self.x = float(a['xcenter'][0:1])
                print("*************")
                print(a['xcenter'])
                self.y = float(a['ycenter'][0:1])
                cv.line(frame,
                        (int(self.x), int(self.y)), (320, 240), (255, 0, 0), 2)
                cv.rectangle(frame, (int(self.x) - int(a['width'][0:1]) // 2,
                                     int(self.y) - int(a['height'][0:1]) // 2), (
                             int(self.x) + int(a['width'][0:1]) // 2,
                             int(self.y) + int(a['height'][0:1]) // 2), kirmizi,
                             3)  # Hedef vuruÅŸ alanÄ±nÄ± Ã§iz
                fps_label = "FPS: %.2f" % (1 / (end_time - start_time))
                cv.putText(frame, fps_label, (0, 25), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                cv.putText(frame, chasing_uav, (10, 360), cv.FONT_HERSHEY_SIMPLEX, 1, beyaz, 1, cv.LINE_AA, False)
                self.yollama(self.x, self.y, True)

        else:
            cv.putText(frame, searching_for_uav, (10, 360), cv.FONT_HERSHEY_SIMPLEX, 1, beyaz, 1, cv.LINE_AA, False)
            self.yollama(0, 0, False)
        fps_label = "FPS: %.2f" % (1 / (end_time - start_time))
        cv.putText(frame, fps_label, (0, 25), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv.imshow("Kuzgun Iha", frame)
        cv.waitKey(1)

nesne = RobotKamera()
