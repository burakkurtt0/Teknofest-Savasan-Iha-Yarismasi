#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
from offb.msg import kamera
from offb.msg import pozisyon
from offb.msg import Num

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
import time
import message_filters
import sys
from geometry_msgs.msg import Pose,  Pose2D
from offb.msg import RL_verisi


"""

rospy.init_node("esleme_dugumuuuu")
model_pozisyon=message_filters.Subscriber('/gazebo/model_states', Pose)
kamera_verisi=message_filters.Subscriber('/yapay_zeka', kamera_listesi)


ts = message_filters.TimeSynchronizer([pozisyon(model_pozisyon), kamera_listesi(kamera_verisi)], 10)
ts.registerCallback(callback)
rospy.spin()

"""
pub = rospy.Publisher('/RL_veri_girisi', RL_verisi, queue_size=10)



veri=RL_verisi()
def callback(data1,data2):
    veri.header.stamp=rospy.Time.now()
    veri.header.frame_id="RL_giris_verisi"
    veri.h=data2.h
    veri.w=data2.w
    veri.alan=data2.alan
    veri.isPlane=data2.isPlane
    veri.x=data1.x
    veri.y=data1.y
    veri.z=data1.z
    veri.yaw=data1.yaw
    veri.rol=data1.rol
    veri.quaternion=data1.w
    rospy.loginfo(veri)
    pub.publish(veri)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('esleme_dugumu111', anonymous=True)

    model_pozisyon=message_filters.Subscriber('/deneme', Num)
    kamera_verisi=message_filters.Subscriber('/yapay_zeka', kamera)

    ts = message_filters.TimeSynchronizer([model_pozisyon, kamera_verisi], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/RL_veri_girisi', RL_verisi, queue_size=1)
    listener()