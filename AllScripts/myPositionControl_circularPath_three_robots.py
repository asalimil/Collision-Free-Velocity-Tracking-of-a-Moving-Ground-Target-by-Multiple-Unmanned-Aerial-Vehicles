#!/usr/bin/env python

import rospy
import tf
import math
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread


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
    def takeOff(self, H):    
    	#
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
	while not rospy.is_shutdown():
	 	while t < t_final:
	              self.msg.x = x0 + 0.25*math.cos(t)
	              self.msg.y = y0 + 0.25*math.sin(t)
	              self.msg.yaw = yaw0
	              self.msg.z = z0
	              now = rospy.get_time()
	              if (now - start > t_final):
	        	 break
	    	      #
	    	      t = t + 0.1
	              self.pub.publish(self.msg)
	              self.rate.sleep()
	    # self.msg.x = x_des
	    # self.msg.y = y_des
	    # self.msg.yaw = yaw_des
	    # self.msg.z = z_des
	    # self.pub.publish(self.msg)
	    # self.rate.sleep()

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



def handler(cf):
    x_des = 0.0; y_des = 0.0; z_des = 0.5
    cf.takeOff(z_des)
    #	
    # t = 0; t_final = 30
    # while t < t_final:
    #	x_des = 0.25*math.cos(t)
    # 	y_des = 0.25*math.sin(t)
    #	z_des = 0.5
    #	#
    #	cf.goTo(x_des, y_des, z_des, 0.0)
    #	t = t + 0.1		
    cf.goTo(x_des, y_des, z_des, 0.0)
    cf.land(H)



if __name__ == '__main__':
    rospy.init_node('cf_node', anonymous=True)

    cf1 = Crazyflie("cf1")
    cf2 = Crazyflie("cf2")
    cf3 = Crazyflie("cf3")    

    t1 = Thread(target=handler, args=(cf1,))
    t2 = Thread(target=handler, args=(cf2,))
    t3 = Thread(target=handler, args=(cf3,))    
    t1.start()
    t2.start()
    t3.start()














