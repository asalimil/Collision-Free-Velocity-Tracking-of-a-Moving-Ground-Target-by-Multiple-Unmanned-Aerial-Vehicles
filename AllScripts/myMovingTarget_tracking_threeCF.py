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
    # stop publishing
    stop_pub_cf1 = rospy.Publisher("cf1/cmd_stop", Empty, queue_size=1)
    stop_msg_cf1 = Empty()
    stop_pub_cf2 = rospy.Publisher("cf2/cmd_stop", Empty, queue_size=1)
    stop_msg_cf2 = Empty()
    stop_pub_cf3 = rospy.Publisher("cf3/cmd_stop", Empty, queue_size=1)
    stop_msg_cf3 = Empty()
    

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
        H = 20
        for h in range(H):
            #
            msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = h/D
            msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = h/D
            msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = h/D
	    #
            now = rospy.get_time()
            pub_position_cf1.publish(msg_cf1)
            # rate.sleep()
            pub_position_cf2.publish(msg_cf2)
            # rate.sleep()
            pub_position_cf3.publish(msg_cf3)
            rate.sleep()
	    #
        for h in range(H):
            msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = H/D
            msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = H/D
            msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = H/D
	    #
            pub_position_cf1.publish(msg_cf1)
            # rate.sleep()
            pub_position_cf2.publish(msg_cf2)
            # rate.sleep()
            pub_position_cf3.publish(msg_cf3)
            rate.sleep()
            #
        break


    
    #---------------------------------------------------------------------------
    # tracking
    start = rospy.get_time()
    k1 = 1.0; k2 = 1.0; k3 = 0.025;
    a = 0.25; w = 1.0; m = 0.050
    t = 0.0; t_final = 30.0; dt = 0.05
    #
    x1_0 = 0.5; y1_0 = 0.5
    x2_0 = 0.5; y2_0 = -0.5
    x3_0 = 0.75; y3_0 = 0.0
    # we manually set cfs' initial position after take off
    x1 = numpy.array([x1_0, y1_0, H/D])
    x1_dot = numpy.array([0.0, 0.0, 0.0])
    x2 = numpy.array([x2_0, y2_0, H/D])
    x2_dot = numpy.array([0.0, 0.0, 0.0])
    x3 = numpy.array([x3_0, y3_0, H/D])
    x3_dot = numpy.array([0.0, 0.0, 0.0])
    #
    while not rospy.is_shutdown():
	while t < t_final:
	      #
              x_tb3 = a*math.cos(w*t); xdot_tb3 = -a*w*math.sin(w*t); xddot_tb3 = -a*w*w*math.cos(w*t)
	      y_tb3 = a*math.sin(w*t); ydot_tb3 = +a*w*math.cos(w*t); yddot_tb3 = -a*w*w*math.sin(w*t)
	      z_des = H/D; zdot_des = 0.0; zddot_des = 0.0
	      #
              x_ref = numpy.array([x_tb3, y_tb3, z_des])
	      xdot_ref = numpy.array([xdot_tb3, ydot_tb3, zdot_des])
	      xddot_ref = numpy.array([xddot_tb3, yddot_tb3, zddot_des])
	      #
              # potential function >> Vi = ln^2(r_ij) + 1/r_ij
              # grad_Vi = (2*ln()/d_ij - 1/d_ij^2)*uv_ij,  uv_ij: unit vector along connecting segment i -> j 
              r_12 = x1 - x2; d_12 = numpy.linalg.norm(r_12); uv_12 = r_12/d_12 
              r_21 = -r_12; d_21 = d_12; uv_21 = -uv_12 
              r_13 = x1 - x3; d_13 = numpy.linalg.norm(r_13); uv_13 = r_13/d_13 
              r_31 = -r_13; d_31 = d_13; uv_31 = -uv_13 
              r_23 = x2 - x3; d_23 = numpy.linalg.norm(r_23); uv_23 = r_23/d_23 
              r_32 = -r_23; d_32 = d_23; uv_32 = -uv_23 
              # 
              p = 2.0
              if d_12 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_13, 2))*uv_13
                   grad_V2 = (1 - p/numpy.power(d_23, 2))*uv_23
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_32, 2))*uv_32
              elif d_13 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12
                   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_23, 2))*uv_23 
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31                      
              elif d_23 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_13, 2))*uv_13 
                   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31
              else:
	      	   grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_13, 2))*uv_13
                   # (2*numpy.log(d_12)/d_12 - 1/numpy.power(d_12, 2))*uv_12		   
	      	   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_23, 2))*uv_23 
                   # (2*numpy.log(d_12)/d_12 - 1/numpy.power(d_12, 2))*uv_12
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_32, 2))*uv_32 
                   # (2*numpy.log(d_21)/d_21 - 1/numpy.power(d_21, 2))*uv_21
	      


              e1 = x1 - x_ref; e1_dot = x1_dot - xdot_ref
              e2 = x2 - x_ref; e2_dot = x2_dot - xdot_ref
              e3 = x3 - x_ref; e3_dot = x3_dot - xdot_ref
              #
	      u1 = m*xddot_ref - k1*e1 - k2*e1_dot - k3*grad_V1	
	      u2 = m*xddot_ref - k1*e2 - k2*e2_dot - k3*grad_V2	
	      u3 = m*xddot_ref - k1*e3 - k2*e3_dot - k3*grad_V3	
	      #
	      x1_dot = x1_dot + u1*dt
	      x2_dot = x2_dot + u2*dt
	      x3_dot = x3_dot + u3*dt
	      #
	      x1 = x1 + x1_dot*dt
	      x2 = x2 + x2_dot*dt
	      x3 = x3 + x3_dot*dt
	      #
              # global >> local positions 
              x1_local = x1 - numpy.array([x1_0, y1_0, 0.0]) 
              x2_local = x2 - numpy.array([x2_0, y2_0, 0.0])    
              x3_local = x3 - numpy.array([x3_0, y3_0, 0.0])    
              #
              msg_cf1.x = x1_local[0]; msg_cf1.y = x1_local[1]; msg_cf1.z = z_des
              msg_cf2.x = x2_local[0]; msg_cf2.y = x2_local[1]; msg_cf2.z = z_des
              msg_cf3.x = x3_local[0]; msg_cf3.y = x3_local[1]; msg_cf3.z = z_des
	      #
              pub_position_cf1.publish(msg_cf1)
              # rate.sleep()
              pub_position_cf2.publish(msg_cf2)
              # rate.sleep()
              pub_position_cf3.publish(msg_cf3)
              rate.sleep()
	      #
              # print("Vij: ", V12, V13, V23)
	      # 
	      t = t + dt
	      now = rospy.get_time()
              if (now - start > 30.0):
                 break

              # rate.sleep()

    
    #---------------------------------------------------------------------------	
    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = 0.0
        msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = 0.0
        msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = 0.0
	#
        now = rospy.get_time()
        if (now - start > 1.0):
            break

        pub_position_cf1.publish(msg_cf1)
        pub_position_cf2.publish(msg_cf2)
        pub_position_cf3.publish(msg_cf3)
        rate.sleep()
    #---------------------------------------------------------------------------


    stop_pub_cf1.publish(stop_msg_cf1)
    stop_pub_cf2.publish(stop_msg_cf2)
    stop_pub_cf3.publish(stop_msg_cf3)



