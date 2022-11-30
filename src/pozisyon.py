#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from offb.msg import kamera_verisi
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GlobalPositionTarget
#/uav1/mavros/global_position/local
from sensor_msgs.msg import NavSatFix
import time

class pozisyon():
    def __init__(self):
        rospy.init_node('posizyonkontrol', anonymous=True)

        rospy.Subscriber('/kamera_verisi', kamera_verisi, self.kamera)
        self.pos_set_point_pub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.i=1
        rospy.spin()



    def kamera(self,data):
        veri = kamera_verisi()
        veri=data
        print(veri.isPlane)
        if veri.isPlane == False:
            print("pozisyon yolladım")
            position_target=rospy.wait_for_message("/uav1/mavros/global_position/local", Odometry)
            self.send_setpoint(position_target)
        else:
            print("pozisyon yollamıyorum")

            time.sleep(0.2)


    def send_setpoint(self,data):
        print("deneme")
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.header.stamp = rospy.Time.now()
        self.i=self.i+1
        pos.header.seq = self.i
        pos.pose.position.x=data.pose.pose.position.x
        pos.pose.position.y=data.pose.pose.position.y
        pos.pose.position.z=data.pose.pose.position.z
        #pos.pose.orientation.x = 0
        #pos.pose.orientation.y = 0
        #pos.pose.orientation.z = 0
        #pos.pose.orientation.w = 1
        #time.sleep(0.1)
        self.pos_set_point_pub.publish(pos)
        #rospy.loginfo(pos)

        print(pos)




uav=pozisyon()












