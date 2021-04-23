#!/usr/bin/env python

import rospy
import tf
import numpy
import math
# cf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
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

"""
def callback_imu(imu):
    global acc_cf
    acc_x = imu.linear_acceleration.x
    acc_y = imu.linear_acceleration.y    
    acc_z = imu.linear_acceleration.z    
    acc_cf = numpy.array([acc_x, acc_y, acc_z])
"""

# initial tb3
x0 = -1.5; y0 = 0.0; th0 = 0.0*math.pi/2
# x0 = 0.0; y0 = 0.0; th0 = 0.0*math.pi/2
X0 = numpy.array([x0, y0, th0])
X_local = numpy.array([0.0,0.0, 0.0])
acc_cf = numpy.array([0.0,0.0, 0.0])
vel_cf = numpy.array([0.0,0.0, 0.0])
x_cf = numpy.array([0.0,0.0, 0.0])


#--- MAIN ---
if __name__ == '__main__':
    #	
    rospy.init_node('cf_node', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) # 10 hz


    # tb3 cmd_vel publisher
    pub_vel_tb3 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    move_tb3 = Twist()    
    # tb3 odometry subscriber
    sub_odom_tb3 = rospy.Subscriber('/tb3_1/odom', Odometry, callback_odom)


    # command position publishers
    pub_position_cf1 = rospy.Publisher("cf1/cmd_position", Position, queue_size=1)
    msg_position_cf1 = Position()
    # command hover publishers
    # pub_hover_cf1 = rospy.Publisher("cf1/cmd_hover", Hover, queue_size=1)
    # msg_hover_cf1 = Hover()
    # stop publishing
    pub_stop_cf1 = rospy.Publisher("cf1/cmd_stop", Empty, queue_size=1)
    msg_stop_cf1 = Empty()
    # cf IMU subscriber
    # sub_imu_cf = rospy.Subscriber('cf1/imu', Imu, callback_imu)
    
    
    # rospy.wait_for_service('cf1/update_params')
    # rospy.loginfo("found update_params service")
    # update_params = rospy.ServiceProxy('cf1/update_params', UpdateParams)

    # rospy.set_param("kalman/resetEstimation", 1)
    # update_params(["kalman/resetEstimation"])
    # rospy.sleep(0.1)
    # rospy.set_param("kalman/resetEstimation", 0)
    # update_params(["kalman/resetEstimation"])
    rospy.sleep(0.5)
    


    #---------------------------------------------------------------------------
    # take off
    print("TAKE OFF ...!")
    while not rospy.is_shutdown():
        D = 50.0
        H = 50
        for h in range(H):
            #
            msg_position_cf1.x = 0.0; msg_position_cf1.y = 0.0; msg_position_cf1.z = h/D
            # msg_hover_cf1.yawrate = 0.0
	    #
            now = rospy.get_time()
            pub_position_cf1.publish(msg_position_cf1)
            # pub_hover_cf1.publish(msg_hover_cf1)
            rate.sleep()
	    # 
        break


    
    #---------------------------------------------------------------------------
    # tracking
    start = rospy.get_time()
    k1 = 1.5; k2 = 1.5
    # a = 0.5; w = 0.25; m = 0.050
    # a = 0.5; w = 0.4; 
    m = 0.050
    z_des = H/D
    t = 0.0; t_final = 40.0; dt = 0.05
    #
    x1_0 = -1.5; y1_0 = +1.0

    # we manually set cfs' initial position after take off
    x1 = numpy.array([x1_0, y1_0, H/D])
    x1_dot = numpy.array([0.0, 0.0, 0.0])
    
    # tb3 cmd_vel
    r = 0.25
    v_tb3 = 0.05; w_tb3 = 0.0 # v_tb3/r # Circular path r = 0.5
    
    # while not rospy.is_shutdown():
    print("TRACKING ...!")
    while t < t_final:
 
          # virtual trajectory
          # x_tb3 = a*math.cos(w*t); xdot_tb3 = -a*w*math.sin(w*t); xddot_tb3 = -a*w*w*math.cos(w*t)
	  # y_tb3 = a*math.sin(w*t); ydot_tb3 = +a*w*math.cos(w*t); yddot_tb3 = -a*w*w*math.sin(w*t)

          
	  # real robot trajectory (tb3)
	  #--->    
          move_tb3.linear.x = v_tb3
          move_tb3.angular.z = w_tb3
          pub_vel_tb3.publish(move_tb3)	      

	                               
          xr = X_local[0] + X0[0]
    	  yr = X_local[1] + X0[1]
          thr = X_local[2]
          X_tb3 = numpy.array([xr, yr, thr])

          Xdot_tb3 = numpy.array([v_tb3*numpy.cos(thr), v_tb3*numpy.sin(thr), 0.0])
          Xddot_tb3 = numpy.array([-v_tb3*w_tb3*numpy.sin(thr), +v_tb3*w_tb3*numpy.cos(thr), 0.0])

	  x_tb3 = X_tb3[0]; xdot_tb3 = Xdot_tb3[0]; xddot_tb3 = Xddot_tb3[0]
          y_tb3 = X_tb3[1]; ydot_tb3 = Xdot_tb3[1]; yddot_tb3 = Xddot_tb3[1]
	  z_des = H/D; zdot_des = 0.0; zddot_des = 0.0
          #<---    

          # print(X_tb3)
	  #
          
          x_ref = numpy.array([x_tb3, y_tb3, z_des])
	  xdot_ref = numpy.array([xdot_tb3, ydot_tb3, 0.0])
	  xddot_ref = numpy.array([xddot_tb3, yddot_tb3, 0.0])
	  #


          e1 = x1 - x_ref; e1_dot = x1_dot - xdot_ref
          #
	  u1 = m*xddot_ref - k1*e1 - k2*e1_dot	
	  #
	  x1_dot = x1_dot + u1*dt
	  #
	  x1 = x1 + x1_dot*dt
	  #
          # global >> local positions 
          x1_local = x1 - numpy.array([x1_0, y1_0, 0.0]) 
          #
          msg_position_cf1.x = x1_local[0]; msg_position_cf1.y = x1_local[1]; msg_position_cf1.z = z_des
	  #
          pub_position_cf1.publish(msg_position_cf1)
	  # 
	  t = t + dt
          #
          print(round(t,3), round(x_ref[0],3) , round(x_ref[1],3), round(x1[0],3), round(x1[1],3), round(u1[0],3), round(u1[1],3), round(u1[2],3))
          # vel_cf = vel_cf + (-acc_cf - numpy.array([0.0, 0.0, -9.8]))*dt
          # x_cf = x_cf + vel_cf*dt
	  # print(x_cf)

	  now = rospy.get_time()
          # if (now - start > t_final):
          #   break

          if t > t_final:
             break

          rate.sleep()

    
    #---------------------------------------------------------------------------	
    # land, spend 1 secs
    # start = rospy.get_time()
    # while not rospy.is_shutdown():
    D = 50.0; H = 50
    print("LANDING ...!")
    for h in range(H):
        zz = (H-h)/D
        msg_position_cf1.x = x1_local[0]; msg_position_cf1.y = x1_local[1]; msg_position_cf1.z = zz
	#
        # now = rospy.get_time()
        if (zz < 0.1):
            break

        pub_position_cf1.publish(msg_position_cf1)
        rate.sleep()
    #---------------------------------------------------------------------------

    pub_stop_cf1.publish(msg_stop_cf1)


