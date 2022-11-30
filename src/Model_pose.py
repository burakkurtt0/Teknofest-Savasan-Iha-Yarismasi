#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#from distutils.log import info
import rospy

import time
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import AttitudeTarget
import rospy
from std_msgs.msg import Header
import numpy as np
import rospy
import time
from geometry_msgs.msg import PoseStamped, Quaternion

from tf.transformations import quaternion_from_euler


#deneme=rospy.Publisher("mavros/setpoint_position/local", 10)



att=AttitudeTarget()
print("offboard ")

def send_att():
    rate = rospy.Rate(10)  # Hz
    print("offboard")
    att.body_rate = Vector3()
    att.header = Header()
    att.header.frame_id = "base_footprint"
    att.orientation = Quaternion(*quaternion_from_euler(0.25, 0.15,0))
    att.thrust = 0.7
    att.type_mask = 7  # ignore body rate
    while not rospy.is_shutdown():
        att.header.stamp = rospy.Time.now()
        att_setpoint_pub.publish(att)
        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    rospy.init_node('offboard_yayinlayici', anonymous=True)
    att_setpoint_pub = rospy.Publisher('uav0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    try:
        send_att()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass