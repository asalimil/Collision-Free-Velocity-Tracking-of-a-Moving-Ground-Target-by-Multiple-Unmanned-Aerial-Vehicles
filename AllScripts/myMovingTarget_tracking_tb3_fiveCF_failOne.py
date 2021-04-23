#!/usr/bin/env python

import rospy
import tf
import numpy
import math
# cf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
# from crazyflie_driver.msg import Hover
# tb3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


# tb3 odometry
def callback_odom(odom):
    global X_local
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    th = 2*numpy.arcsin(qz)
    X_local = numpy.array([x, y, th])


# initial tb3
x0 = 0.0; y0 = -0.25; th0 = 0.0*math.pi/2
# x0 = 0.0; y0 = 0.0; th0 = 0.0*math.pi/2
X0 = numpy.array([x0, y0, th0])
X_local = numpy.array([0.0,0.0, 0.0])


#--- MAIN ---
if __name__ == '__main__':
    #	
    rospy.init_node('multi_cf_node', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) # 10 hz

    # tb3 cmd_vel publisher
    pub_vel_tb3 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    move_tb3 = Twist()    
    # tb3 odometry subscriber
    sub_odom_tb3 = rospy.Subscriber('/tb3_1/odom', Odometry, callback_odom)


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
    

    rospy.set_param("cf1/kalman/resetEstimation", 1)
    update_params(["cf1/kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("cf1/kalman/resetEstimation", 0)
    update_params(["cf1/kalman/resetEstimation"])
    rospy.sleep(0.1)
    #
    rospy.set_param("cf2/kalman/resetEstimation", 1)
    update_params(["cf2/kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("cf2/kalman/resetEstimation", 0)
    update_params(["cf2/kalman/resetEstimation"])
    rospy.sleep(0.1)
    #
    rospy.set_param("cf3/kalman/resetEstimation", 1)
    update_params(["cf3/kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("cf3/kalman/resetEstimation", 0)
    update_params(["cf3/kalman/resetEstimation"])
    rospy.sleep(0.1)
    #
    rospy.set_param("cf4/kalman/resetEstimation", 1)
    update_params(["cf4/kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("cf4/kalman/resetEstimation", 0)
    update_params(["cf4/kalman/resetEstimation"])
    rospy.sleep(0.1)
    #
    rospy.set_param("cf5/kalman/resetEstimation", 1)
    update_params(["cf5/kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("cf5/kalman/resetEstimation", 0)
    update_params(["cf5/kalman/resetEstimation"])
    rospy.sleep(0.1)
    #


    # rospy.wait_for_service('update_params')
    # rospy.loginfo("found update_params service")
    # update_params = rospy.ServiceProxy('update_params', UpdateParams)

    # rospy.set_param("kalman/resetEstimation", 1)
    # update_params(["kalman/resetEstimation"])
    # rospy.sleep(0.1)
    # rospy.set_param("kalman/resetEstimation", 0)
    # update_params(["kalman/resetEstimation"])
    # rospy.sleep(0.5)
    rospy.sleep(0.5)


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
            pub_position_cf2.publish(msg_cf2)
            pub_position_cf3.publish(msg_cf3)
            pub_position_cf4.publish(msg_cf4)
            pub_position_cf5.publish(msg_cf5)
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
            pub_position_cf2.publish(msg_cf2)
            pub_position_cf3.publish(msg_cf3)
            pub_position_cf4.publish(msg_cf4)
            pub_position_cf5.publish(msg_cf5)
            rate.sleep()
            #
        break


    
    #---------------------------------------------------------------------------
    # tracking
    start = rospy.get_time()
    k1 = 1.0; k2 = 1.0; k3 = 0.02;
    # k1 = 1.0; k2 = 1.25; k3 = 0.035
    a = 1.0; w = 0.25; m = 0.050
    t = 0.0; t_final = 50.0; dt = 0.05
    #
    x1_0 = +0.50; y1_0 = -0.50
    x2_0 = +0.00; y2_0 = -0.50
    x3_0 = -0.50; y3_0 = -0.50
    x4_0 = +0.50; y4_0 = +0.00
    x5_0 = +0.50; y5_0 = +0.50
    # x1_0 = +2.0; y1_0 = +0.0
    # x2_0 = -2.0; y2_0 = +0.0
    # x3_0 = +0.0; y3_0 = +2.0

    # we manually set cfs' initial position after take off
    x1 = numpy.array([x1_0, y1_0, H/D])
    x1_dot = numpy.array([0.0, 0.0, 0.0])
    x2 = numpy.array([x2_0, y2_0, H/D])
    x2_dot = numpy.array([0.0, 0.0, 0.0])
    x3 = numpy.array([x3_0, y3_0, H/D])
    x3_dot = numpy.array([0.0, 0.0, 0.0])
    x4 = numpy.array([x4_0, y4_0, H/D])
    x4_dot = numpy.array([0.0, 0.0, 0.0])
    x5 = numpy.array([x5_0, y5_0, H/D])
    x5_dot = numpy.array([0.0, 0.0, 0.0])

    # tb3 cmd_vel
    radius = 0.25
    v_tb3 = 0.1; w_tb3 = v_tb3/radius # Circular path r = 0.25
    # v_tb3 = 0.15/3; w_tb3 = 0 # Straight path
    #
    while not rospy.is_shutdown():
	while t < t_final:
	      # move tb3
              move_tb3.linear.x = v_tb3
              move_tb3.angular.z = w_tb3
              pub_vel_tb3.publish(move_tb3)	      

	      # trajectory virtual tb3
              # x_tb3 = a*math.cos(w*t); xdot_tb3 = -a*w*math.sin(w*t); xddot_tb3 = -a*w*w*math.cos(w*t)
	      # y_tb3 = a*math.sin(w*t); ydot_tb3 = +a*w*math.cos(w*t); yddot_tb3 = -a*w*w*math.sin(w*t)
               
              # trajectory of real tb3
              xr = X_local[0] + X0[0]
    	      yr = X_local[1] + X0[1]
              thr = X_local[2]
              X_tb3 = numpy.array([xr, yr, thr])

              Xdot_tb3 = numpy.array([v_tb3*numpy.cos(thr), v_tb3*numpy.sin(thr), 0.0])
              Xddot_tb3 = numpy.array([-v_tb3*w_tb3*numpy.sin(thr), +v_tb3*w_tb3*numpy.cos(thr), 0.0])

	      x_tb3 = X_tb3[0]; xdot_tb3 = Xdot_tb3[0]; xddot_tb3 = Xddot_tb3[0]
              y_tb3 = X_tb3[1]; ydot_tb3 = Xdot_tb3[1]; yddot_tb3 = Xddot_tb3[1]
	      z_des = H/D; zdot_des = 0.0; zddot_des = 0.0

              # print(X_tb3)
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
              r_14 = x1 - x4; d_14 = numpy.linalg.norm(r_14); uv_14 = r_14/d_14 
              r_41 = -r_14; d_41 = d_14; uv_41 = -uv_14 
              r_15 = x1 - x5; d_15 = numpy.linalg.norm(r_15); uv_15 = r_15/d_15 
              r_51 = -r_15; d_51 = d_15; uv_51 = -uv_15
              r_24 = x2 - x4; d_24 = numpy.linalg.norm(r_24); uv_24 = r_24/d_24 
              r_42 = -r_24; d_42 = d_24; uv_42 = -uv_24
              r_25 = x2 - x5; d_25 = numpy.linalg.norm(r_25); uv_25 = r_25/d_25 
              r_52 = -r_25; d_52 = d_25; uv_52 = -uv_25
              r_34 = x3 - x4; d_34 = numpy.linalg.norm(r_34); uv_34 = r_34/d_34 
              r_43 = -r_34; d_43 = d_34; uv_43 = -uv_34
              r_35 = x3 - x5; d_35 = numpy.linalg.norm(r_35); uv_35 = r_35/d_35 
              r_53 = -r_35; d_53 = d_35; uv_53 = -uv_35 
              r_45 = x4 - x5; d_45 = numpy.linalg.norm(r_45); uv_45 = r_45/d_45 
              r_54 = -r_45; d_54 = d_45; uv_54 = -uv_45              
	      # 
              p = numpy.sqrt(a)
              grad_V1 = (1-p/numpy.power(d_12,2))*uv_12+(1-p/numpy.power(d_13,2))*uv_13+(1-p/numpy.power(d_14,2))*uv_14+(1-p/numpy.power(d_15,2))*uv_15 
	      grad_V2 = (1-p/numpy.power(d_21,2))*uv_21+(1-p/numpy.power(d_23,2))*uv_23+(1-p/numpy.power(d_24,2))*uv_24+(1-p/numpy.power(d_25,2))*uv_25 
              grad_V3 = (1-p/numpy.power(d_31,2))*uv_31+(1-p/numpy.power(d_32,2))*uv_32+(1-p/numpy.power(d_34,2))*uv_34+(1-p/numpy.power(d_35,2))*uv_35 
              grad_V4 = (1-p/numpy.power(d_41,2))*uv_41+(1-p/numpy.power(d_42,2))*uv_42+(1-p/numpy.power(d_43,2))*uv_43+(1-p/numpy.power(d_45,2))*uv_45 
              grad_V5 = (1-p/numpy.power(d_51,2))*uv_51+(1-p/numpy.power(d_52,2))*uv_52+(1-p/numpy.power(d_53,2))*uv_53+(1-p/numpy.power(d_54,2))*uv_54 
	      #
              if d_12 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_13, 2))*uv_13 + (1 - p/numpy.power(d_14, 2))*uv_14 + (1 - p/numpy.power(d_15, 2))*uv_15 
                   grad_V2 = (1 - p/numpy.power(d_23, 2))*uv_23 + (1 - p/numpy.power(d_24, 2))*uv_24 + (1 - p/numpy.power(d_25, 2))*uv_25 
              elif d_13 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_14, 2))*uv_14 + (1 - p/numpy.power(d_15, 2))*uv_15 
                   grad_V3 = (1 - p/numpy.power(d_32, 2))*uv_32 + (1 - p/numpy.power(d_34, 2))*uv_34 + (1 - p/numpy.power(d_35, 2))*uv_35
              elif d_23 > numpy.sqrt(p):
                   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_24, 2))*uv_24 + (1 - p/numpy.power(d_25, 2))*uv_25 
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_34, 2))*uv_34 + (1 - p/numpy.power(d_35, 2))*uv_35
              elif d_14 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_13, 2))*uv_13 + (1 - p/numpy.power(d_15, 2))*uv_15
                   grad_V4 = (1 - p/numpy.power(d_42, 2))*uv_42 + (1 - p/numpy.power(d_43, 2))*uv_43 + (1 - p/numpy.power(d_45, 2))*uv_45 
              elif d_15 > numpy.sqrt(p):
	           grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_13, 2))*uv_13 + (1 - p/numpy.power(d_14, 2))*uv_14
                   grad_V5 = (1 - p/numpy.power(d_52, 2))*uv_52 + (1 - p/numpy.power(d_53, 2))*uv_53 + (1 - p/numpy.power(d_54, 2))*uv_54
              elif d_24 > numpy.sqrt(p):
                   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_23, 2))*uv_23 + (1 - p/numpy.power(d_25, 2))*uv_25
                   grad_V4 = (1 - p/numpy.power(d_41, 2))*uv_41 + (1 - p/numpy.power(d_43, 2))*uv_43 + (1 - p/numpy.power(d_45, 2))*uv_45
              elif d_25 > numpy.sqrt(p):
                   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_23, 2))*uv_23 + (1 - p/numpy.power(d_24, 2))*uv_24
                   grad_V5 = (1 - p/numpy.power(d_51, 2))*uv_51 + (1 - p/numpy.power(d_53, 2))*uv_53 + (1 - p/numpy.power(d_54, 2))*uv_54
              elif d_34 > numpy.sqrt(p):
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_32, 2))*uv_32 + (1 - p/numpy.power(d_35, 2))*uv_35
                   grad_V4 = (1 - p/numpy.power(d_41, 2))*uv_41 + (1 - p/numpy.power(d_42, 2))*uv_42 + (1 - p/numpy.power(d_45, 2))*uv_45 
              elif d_35 > numpy.sqrt(p):
                   grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_32, 2))*uv_32 + (1 - p/numpy.power(d_34, 2))*uv_34
                   grad_V5 = (1 - p/numpy.power(d_51, 2))*uv_51 + (1 - p/numpy.power(d_52, 2))*uv_52 + (1 - p/numpy.power(d_54, 2))*uv_54
              elif d_45 > numpy.sqrt(p):
                   grad_V4 = (1 - p/numpy.power(d_41, 2))*uv_41 + (1 - p/numpy.power(d_42, 2))*uv_42 + (1 - p/numpy.power(d_43, 2))*uv_43 
                   grad_V5 = (1 - p/numpy.power(d_51, 2))*uv_51 + (1 - p/numpy.power(d_52, 2))*uv_52 + (1 - p/numpy.power(d_53, 2))*uv_53
              # else:
	      #	   grad_V1 = (1 - p/numpy.power(d_12, 2))*uv_12 + (1 - p/numpy.power(d_13, 2))*uv_13
              #     # (2*numpy.log(d_12)/d_12 - 1/numpy.power(d_12, 2))*uv_12		   
	      #	   grad_V2 = (1 - p/numpy.power(d_21, 2))*uv_21 + (1 - p/numpy.power(d_23, 2))*uv_23 
              #     # (2*numpy.log(d_12)/d_12 - 1/numpy.power(d_12, 2))*uv_12
              #     grad_V3 = (1 - p/numpy.power(d_31, 2))*uv_31 + (1 - p/numpy.power(d_32, 2))*uv_32 
              #     # (2*numpy.log(d_21)/d_21 - 1/numpy.power(d_21, 2))*uv_21
	      


              e1 = x1 - x_ref; e1_dot = x1_dot - xdot_ref
              e2 = x2 - x_ref; e2_dot = x2_dot - xdot_ref
              e3 = x3 - x_ref; e3_dot = x3_dot - xdot_ref
              e4 = x4 - x_ref; e4_dot = x4_dot - xdot_ref
              e5 = x5 - x_ref; e5_dot = x5_dot - xdot_ref
              #
	      u1 = m*xddot_ref - k1*e1 - k2*e1_dot - k3*grad_V1	
	      u2 = m*xddot_ref - k1*e2 - k2*e2_dot - k3*grad_V2	
	      u3 = m*xddot_ref - k1*e3 - k2*e3_dot - k3*grad_V3	
	      u4 = m*xddot_ref - k1*e4 - k2*e4_dot - k3*grad_V4	
	      u5 = m*xddot_ref - k1*e5 - k2*e5_dot - k3*grad_V5	
	      #
	      x1_dot = x1_dot + u1*dt
	      x2_dot = x2_dot + u2*dt
	      x3_dot = x3_dot + u3*dt
	      x4_dot = x4_dot + u4*dt
	      x5_dot = x5_dot + u5*dt
	      #
	      x1 = x1 + x1_dot*dt
	      x2 = x2 + x2_dot*dt
	      x3 = x3 + x3_dot*dt
	      x4 = x4 + x4_dot*dt
	      x5 = x5 + x5_dot*dt
	      #
              # global >> local positions 
              x1_local = x1 - numpy.array([x1_0, y1_0, 0.0]) 
              x2_local = x2 - numpy.array([x2_0, y2_0, 0.0])    
              x3_local = x3 - numpy.array([x3_0, y3_0, 0.0])    
              x4_local = x4 - numpy.array([x4_0, y4_0, 0.0])    
              x5_local = x5 - numpy.array([x5_0, y5_0, 0.0])    
              #
              msg_cf1.x = x1_local[0]; msg_cf1.y = x1_local[1]; msg_cf1.z = z_des
              msg_cf2.x = x2_local[0]; msg_cf2.y = x2_local[1]; msg_cf2.z = z_des
              msg_cf3.x = x3_local[0]; msg_cf3.y = x3_local[1]; msg_cf3.z = z_des
              msg_cf4.x = x4_local[0]; msg_cf4.y = x4_local[1]; msg_cf4.z = z_des
              msg_cf5.x = x5_local[0]; msg_cf5.y = x5_local[1]; msg_cf5.z = z_des
	      #
              pub_position_cf1.publish(msg_cf1); rospy.sleep(0.1)
              pub_position_cf2.publish(msg_cf2); rospy.sleep(0.1)
              pub_position_cf3.publish(msg_cf3); rospy.sleep(0.1)
              pub_position_cf4.publish(msg_cf4); rospy.sleep(0.1)
              pub_position_cf5.publish(msg_cf5); rospy.sleep(0.1)
	      #
              # print("Vij: ", V12, V13, V23)
	      # 
	      t = t + dt
	      now = rospy.get_time()
              if (now - start > t_final):
                 break

              # rate.sleep()

    
    #---------------------------------------------------------------------------	
    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        for h in range(H):
            msg_cf1.x = 0.0; msg_cf1.y = 0.0; msg_cf1.z = H - h/D
            msg_cf2.x = 0.0; msg_cf2.y = 0.0; msg_cf2.z = H - h/D
            msg_cf3.x = 0.0; msg_cf3.y = 0.0; msg_cf3.z = H - h/D
            msg_cf4.x = 0.0; msg_cf4.y = 0.0; msg_cf4.z = H - h/D
            msg_cf5.x = 0.0; msg_cf5.y = 0.0; msg_cf5.z = H - h/D
	    #
            # now = rospy.get_time()
            # if (now - start > 1.0):
            #	break
            #
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



