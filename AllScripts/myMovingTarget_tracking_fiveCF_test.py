#!/usr/bin/env python

import rospy
import tf
import numpy
import math
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    #	
    rospy.init_node('multi_cf_node', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) # 10 hz

    # command position publishers
    pub_position_cf1 = rospy.Publisher("cf1/cmd_position", Position, queue_size=1)
    msg_cf1 = Position()
    pub_position_cf2 = rospy.Publisher("cf2/cmd_position", Position, queue_size=1)
    msg_cf2 = Position()
    pub_position_cf3 = rospy.Publisher("cf3/cmd_position", Position, queue_size=1)
    msg_cf3 = Position()
    pub_position_cf4 = rospy.Publisher("cf4/cmd_position", Position, queue_size=1)
    msg_cf4 = Position()
    pub_position_cf5 = rospy.Publisher("cf5/cmd_position", Position, queue_size=1)
    msg_cf5 = Position()

    # stop publishing
    stop_pub_cf1 = rospy.Publisher("cf1/cmd_stop", Empty, queue_size=1)
    stop_msg_cf1 = Empty()
    stop_pub_cf2 = rospy.Publisher("cf2/cmd_stop", Empty, queue_size=1)
    stop_msg_cf2 = Empty()
    stop_pub_cf3 = rospy.Publisher("cf3/cmd_stop", Empty, queue_size=1)
    stop_msg_cf3 = Empty()
    stop_pub_cf4 = rospy.Publisher("cf4/cmd_stop", Empty, queue_size=1)
    stop_msg_cf4 = Empty()
    stop_pub_cf5 = rospy.Publisher("cf5/cmd_stop", Empty, queue_size=1)
    stop_msg_cf5 = Empty()

    # rospy.wait_for_service('update_params')
    # rospy.loginfo("found update_params service")
    # update_params = rospy.ServiceProxy('update_params', UpdateParams)

    # rospy.set_param("kalman/resetEstimation", 1)
    # update_params(["kalman/resetEstimation"])
    # rospy.sleep(0.1)
    # rospy.set_param("kalman/resetEstimation", 0)
    # update_params(["kalman/resetEstimation"])
    # rospy.sleep(0.5)
    rospy.sleep(1.0)


    #---------------------------------------------------------------------------
    # take off
    while not rospy.is_shutdown():
        D = 50.0
        H = 50
        for h in range(H):
            #
            msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = h/D
            msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = h/D
            msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = h/D
            msg_cf4.x = 0.0; msg_cf4.y = 0.0; msg_cf4.z = h/D
            msg_cf5.x = 0.0; msg_cf5.y = 0.0; msg_cf5.z = h/D
	    #
            now = rospy.get_time()
            pub_position_cf1.publish(msg_cf1)
            # rate.sleep()
            pub_position_cf2.publish(msg_cf2)
            # rate.sleep()
            pub_position_cf3.publish(msg_cf3)
            # rate.sleep()
            pub_position_cf4.publish(msg_cf4)
            # rate.sleep()
            pub_position_cf5.publish(msg_cf5)
            # rate.sleep()
            rate.sleep()
	    #
        for h in range(H):
            msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = H/D
            msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = H/D
            msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = H/D
            msg_cf4.x = 0.0; msg_cf4.y = 0.0; msg_cf4.z = H/D
            msg_cf5.x = 0.0; msg_cf5.y = 0.0; msg_cf5.z = H/D
	    #
            pub_position_cf1.publish(msg_cf1)
            # rate.sleep()
            pub_position_cf2.publish(msg_cf2)
            # rate.sleep()
            pub_position_cf3.publish(msg_cf3)
            # rate.sleep()
            pub_position_cf4.publish(msg_cf4)
            # rate.sleep()
            pub_position_cf5.publish(msg_cf5)
            # rate.sleep()
            rate.sleep()
            #
        break
    
    #---------------------------------------------------------------------------	
    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = 0.0
        msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = 0.0
        msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = 0.0
        msg_cf4.x = 0.0; msg_cf4.y = 0.0; msg_cf4.z = 0.0
        msg_cf5.x = 0.0; msg_cf5.y = 0.0; msg_cf5.z = 0.0
	#
        now = rospy.get_time()
        if (now - start > 1.0):
            break

        pub_position_cf1.publish(msg_cf1)
        pub_position_cf2.publish(msg_cf2)
        pub_position_cf3.publish(msg_cf3)
        pub_position_cf4.publish(msg_cf4)
        pub_position_cf5.publish(msg_cf5)
        rate.sleep()
    #---------------------------------------------------------------------------


    stop_pub_cf1.publish(stop_msg_cf1)
    stop_pub_cf2.publish(stop_msg_cf2)
    stop_pub_cf3.publish(stop_msg_cf3)
    stop_pub_cf4.publish(stop_msg_cf4)
    stop_pub_cf5.publish(stop_msg_cf5)



