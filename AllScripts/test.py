#!/usr/bin/env python

import rospy
import tf
import numpy
import math
from numpy import linalg as LA
# cf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
# from sensor_msgs.msg import Imu
from crazyflie_driver.srv import UpdateParams
# from crazyflie_driver.msg import FullState



"""
def callback_imu_cf(cf_imu):
    global Acc_lin_cf
    acc_lin_cf_x = cf_imu.linear_acceleration.x
    acc_lin_cf_y = cf_imu.linear_acceleration.y
    acc_lin_cf_z = cf_imu.linear_acceleration.z
    Acc_lin_cf = numpy.array([acc_lin_cf_x, acc_lin_cf_y, acc_lin_cf_z])
    # print(Acc_lin_cf)
"""


# initial conditions tb3
x0_tb3 = 0.0; y0_tb3 = 0.0; th0_tb3 = 0.0
X0_tb3 = numpy.array([x0_tb3, y0_tb3, th0_tb3])
X_tb3 = numpy.array([0.0,0.0, 0.0, 0.0])

# initial conditions cf
Acc_lin_cf = numpy.array([0.0,0.0, 0.0])
v_cf = numpy.array([0.0, 0.0, 0.0])
x_cf = numpy.array([0.0, 0.0, 0.0])
xdot_cf = numpy.array([0.0,0.0, 0.0])
#
dt = 0.05; delta_t = 0.05
k1 = 0.5; k2 = 0.25; k = 0.25


if __name__ == '__main__':
    #
    rospy.init_node('moving_target_node', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) # 10 hz

    # cf
    pub_cf_position = rospy.Publisher("cmd_position", Position, queue_size=1)
    msg_cf_position = Position()
    # pub_cf_acc = rospy.Publisher('/crazyflie/cmd_full_state', FullState, queue_size=10)
    # msg_cf_acc = FullState() 
    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    # sub_imu_cf = rospy.Subscriber('/crazyflie/imu', Imu, callback_imu_cf)

    #
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.5)
    
    """
    # tb3	
    sub_odom = rospy.Subscriber('/tb3_1/odom', Odometry, callback_odom)
    pub_vel_tb3 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    move_tb3 = Twist()	
    """
	
    #===>>
    # take off
    while not rospy.is_shutdown():
        for h in range(10):
            msg_cf_position.x = 0.0
            msg_cf_position.y = 0.0
            msg_cf_position.yaw = 0.0
            msg_cf_position.z = h/20.0
            now = rospy.get_time()
	    #
            pub_cf_position.publish(msg_cf_position)
            rate.sleep()
        for h in range(20):
            msg_cf_position.x = 0.0
            msg_cf_position.y = 0.0
            msg_cf_position.yaw = 0.0
            msg_cf_position.z = 0.5
	    # 
            pub_cf_position.publish(msg_cf_position)
            rate.sleep()
        break


    
    #===>>
    # tracking
    start = rospy.get_time()
    t = 0.0
    while not rospy.is_shutdown():
	# tb3
        # qz = X[2]
        # qw = X[3]
	# thr = 2*numpy.arcsin(qz)
        # Xr = [xr, yr, thr]
        # desired altitudes
        xr = 0.3*math.cos(t) # X[0] + X0[0]
        yr = 0.3*math.sin(t) # X[1] + X0[1]
        z_des = 0.5
	# tb3 position
        x_tb3 = numpy.array([xr, yr, z_des])
	# tb3 linear velocity
	# v_tb3 = 0.1
        # xdot_tb3 = numpy([xr + v_tb3*numpy.cos(thr)*dt, yr + v_tb3*numpy.sin(thr)*dt, 0.0]) 
        xdot_tb3 = numpy.array([-0.3*math.sin(t), 0.3*math.cos(t), 0.0])
	xddot_tb3 = numpy.array([-0.3*math.cos(t), -0.3*math.sin(t), 0.0])
	#
	# cf linear velocity
	# acc_lin_cf = Acc_lin_cf
	# xdot_cf = xdot_cf + acc_lin_cf*dt        
	# controller
        e = x_cf - x_tb3 
        edot = xdot_cf - xdot_tb3 
	S = e + k*edot
        grad_V = 0.0
	u = -k1*S + k2*grad_V + xddot_tb3
	# test
        now = rospy.get_time()
	if (now - start > 50.0):
            break

	# integrate u = xddot >> calculate desired position of cf
        v_cf = v_cf + u*dt
        x_cf = x_cf + v_cf*dt
	# cf publish control cmd
        # # msg_cf_acc.acc.x = u[0]; msg_cf_acc.acc.y = u[1]; msg_cf_acc.acc.z = u[2]
        # # print("t: ", now - start)
	# # pub_cf_acc.publish(msg_cf_acc) 
        msg_cf_position.x = x_cf[0]
        msg_cf_position.y = x_cf[1]
        msg_cf_position.yaw = 0.0
        msg_cf_position.z = 0.5
	pub_cf_position.publish(msg_cf_position)
	# 
	# tb3 publish control cmd
	# move_tb3.x = v_tb3 
	# move_tb3.z = 0.2    
	# pub_vel_tb3.publish(move_tb3)
	#
	t = t + delta_t
        print(" e: ", e, " edot: ", edot)
        rate.sleep()

    

    # land (5 sec)
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg_cf_position.x = 0.0  # current x of drone
        msg_cf_position.y = 0.0  # current y of drone
        msg_cf_position.z = 0.0  
        msg_cf_position.yaw = 0.0  
        now = rospy.get_time()
        if (now - start > 2.0):
            break

	#
        pub_cf_position.publish(msg_cf_position)
        rate.sleep()

    stop_pub.publish(stop_msg)
