#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from mavros_msgs.srv import SetMode,CommandTOL,CommandBool
from gazebo_msgs.msg import ModelState 
from mavros_msgs.srv import SetMode





def setOffboardMode():
    rospy.wait_for_service('uav1/mavros/set_mode', 5)
    try:
        flightModeService = rospy.ServiceProxy('uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
        response = flightModeService(custom_mode='OFFBOARD')
        return response.mode_sent
    except rospy.ServiceException as e:
        print
        "service set_mode call failed: %s. Offboard Mode could not be set." % e
        return False
        
def setLandMode():
    rospy.wait_for_service('uav1/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('uav1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
          
def setArm():
    rospy.wait_for_service('uav0/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        
def setDisarm():
    rospy.wait_for_service('uav0/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e


def setTakeoffMode():
    rospy.wait_for_service('uav1/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('uav1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 101, latitude = 100, longitude = 100, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e


def setResetModel():
    print("reset")
    set_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    state_msg = ModelState()
    state_msg.model_name = 'plane_1'
    state_msg.pose.position.x = 4
    state_msg.pose.position.y = 6
    state_msg.pose.position.z = 0.2
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    set_model.publish(state_msg)
    
    




def menu():
    print "Press"
    print "********************** PLANE 2 *********************************"
    print "1: to set mode to ARM"
    print "2: to set mode to DISARM"
    print "3: to set mode to TAKEOFF"
    print "4: to set mode to LAND"
    print "5: to set mode to OFFBOARD"
    print "6: to set mode to RESET"
    






def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6'])):
        menu()
        x = raw_input("Enter your input: ");
        if (x=='1'):
            setArm()
        elif(x=='2'):
            setDisarm()
        elif(x=='3'):
            setTakeoffMode()
        elif(x=='4'):
            setLandMode()
        elif(x=='5'):
            setOffboardMode ()
        elif(x=='6'):
            setResetModel()

        else: 
            print "Exit"




if __name__ == '__main__':
    rospy.init_node('Kuzgun_iha', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.Rate(10)
    #listener()
    myLoop()
    rospy.spin()
