#!/usr/bin/env python

import rospy
import numpy
import tf
import math

# cf
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread

# tb3
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry

# initial conditions tb3
# x0_tb3 = 0.0; y0_tb3 = 0.0; th0_tb3 = 0.0
# X0_tb3 = numpy.array([x0_tb3, y0_tb3, th0_tb3])
# X_tb3 = numpy.array([0.0,0.0, 0.0, 0.0])

# initial conditions cf
# Acc_lin_cf = numpy.array([0.0,0.0, 0.0])
# v_cf = numpy.array([0.0, 0.0, 0.0])
# x_cf = numpy.array([0.0, 0.0, 0.0])
# xdot_cf = numpy.array([0.0,0.0, 0.0])
#
dt = 0.05; delta_t = 0.05
k1 = 5.0; k2 = 5.0; k3 = 2.0
m = 0.042 # crazyflie weight
w = 0.2*math.pi
a = 0.4


#----------------------------------------
class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)
	
	# update params service
        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)
        self.setParam("kalman/resetEstimation", 1)
	
	# publishers: cmd_position, cmd_stop
        self.pub = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.msg = Position()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        # self.msg.yawrate = 0
        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    # TAKE OFF
    def takeOff(self, z_ref):    
    	#
	H = z_ref
    	resolution = 50 # resolution of z increments 
    	H_scaled = resolution*H
    	#
    	while not rospy.is_shutdown():
        	for h in range(25):
        	    #
        	    self.msg.x = 0.0
        	    self.msg.y = 0.0
        	    self.msg.yaw = 0.0
        	    self.msg.z = h/H_scaled
	            self.pub.publish(self.msg)
	            self.rate.sleep()
		#
                for i in range(20):
		    self.msg.x = 0.0
                    self.msg.y = 0.0
                    self.msg.yaw = 0.0
                    self.msg.z = H
	            self.pub.publish(self.msg)
	            self.rate.sleep()
	        break


    # GOTO position
    # TODO: solve 0
    def goTo (self, x0, y0, z0, yaw0):
	#
	start = rospy.get_time()
	t = 0.0; t_final = 30.0
        x = numpy.array([x0, y0, z0])
        xdot = numpy.array([0.0, 0.0, 0.0])
	#
	while not rospy.is_shutdown():
	 	while t < t_final:
		      x_tb3 = 0.5 + 0.25*math.cos(w*t); xdot_tb3 = -0.25*w*math.sin(w*t); xddot_tb3 = -0.25*w*w*math.cos(w*t)
		      y_tb3 = 0.5 + 0.25*math.sin(w*t); ydot_tb3 = +0.25*w*math.cos(w*t); yddot_tb3 = -0.25*w*w*math.sin(w*t)
		      z_des = +0.5; zdot_des = 0.0; zddot_des = 0.0		            
		      # x_tb3 = a*math.cos(w*t)*math.sin(w*t); xdot_tb3 = a*w*math.cos(2*w*t); xddot_tb3 = -2*a*w*w*math.sin(2*w*t)
		      # y_tb3 = a*math.sin(w*t); ydot_tb3 = a*w*math.cos(w*t); yddot_tb3 = -a*w*w*math.sin(w*t)
		      # z_des = +0.5; zdot_des = 0.0; zddot_des = 0.0		            
		      #	
		      x_ref = numpy.array([x_tb3, y_tb3, z_des])
		      xdot_ref = numpy.array([xdot_tb3, ydot_tb3, zdot_des])
		      xddot_ref = numpy.array([xddot_tb3, yddot_tb3, zddot_des])
    		      #
		      grad_V = 0.0
		      e = x - x_ref; edot = xdot - xdot_ref
		      u = m*xddot_ref - k1*e - k2*edot + k3*grad_V	
		      xdot = xdot + u*dt
		      x = x + xdot*dt
		      print("e: ", e, "edot: ", edot)
		      #
	              self.msg.x = x[0] # x_ref
	              self.msg.y = x[1] # y_ref
	              self.msg.z = x[2] # z_ref
		      #	
	              now = rospy.get_time()
	              if (now - start > t_final):
	        	 break
	    	      
	    	      t = t + 0.05
	              self.pub.publish(self.msg)
	              self.rate.sleep()



    # LANDING
    def land (self, H):
	# land from H to z = 0
	start = rospy.get_time()
	while not rospy.is_shutdown():
		while H > 0:
		      self.msg.x = 0.0
		      self.msg.y = 0.0
		      self.msg.z -= 0.1 
		      self.msg.yaw = 0.0
	              H -= 0.1 
		      self.pub.publish(self.msg)
		      self.rate.sleep()

#----------------------------------------

"""
def callback_odom(odom):
    global X
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    X = numpy.array([x, y, qz, qw])
"""


def handler(cf):
    z_ref = 0.5
    cf.takeOff(z_ref)
    #
    """	
    t = 0; t_final = 50
    while t < t_final:
    	x_tb3 = +0.25*math.cos(t); xdot_tb3 = -0.25*math.sin(t); xddot_tb3 = -0.25*math.cos(t)
     	y_tb3 = +0.25*math.sin(t); ydot_tb3 = +0.25*math.cos(t); yddot_tb3 = -0.25*math.sin(t)
    	z_des = +0.5; zdot_des = 0.0; zddot_des = 0.0
	#
	x_ref = numpy.array([x_tb3, y_tb3, z_des])
	xdot_ref = numpy.array([xdot_tb3, ydot_tb3, zdot_des])
	xddot_ref = numpy.array([xddot_tb3, yddot_tb3, zddot_des])
    	#
        grad_V = 0.0
        e = x - x_ref; edot = xdot - xdot_tb3
	S = e + k3*edot
	u = m*xddot_ref - k1*S + k2*grad_V
	xdot = xdot + u*dt
	x = x + xdot*dt
	print("e: ", e)
	#
    	cf.goTo(x_ref[0], x_ref[1], x_ref[2], 0.0)
    	t = t + 0.1		
    """
    x0 = 0.0; y0 = 0.0; z0 = z_ref
    cf.goTo(x0, y0, z0, 0.0)
    cf.land(H)




#----------------------------------------
if __name__ == '__main__':
    rospy.init_node('multi_UAVs_node', anonymous=True)

    cf1 = Crazyflie("cf1")
    # cf2 = Crazyflie("cf2")
    # cf3 = Crazyflie("cf3")    

    t1 = Thread(target=handler, args=(cf1,))
    # t2 = Thread(target=handler, args=(cf2,))
    # t3 = Thread(target=handler, args=(cf3,))    
    t1.start()
    # t2.start()
    # t3.start()














