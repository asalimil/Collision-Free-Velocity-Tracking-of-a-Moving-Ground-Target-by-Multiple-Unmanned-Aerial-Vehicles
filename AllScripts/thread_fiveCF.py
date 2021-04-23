#!/usr/bin/env python

import rospy
import tf
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

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        # ---> myCodes
        self.pub_position = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.msg_position = Position()
        #

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()


    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])


    # take off to z distance
    def takeOff(self, H):
        D = 50.0   # H/D determines take-off's final altitude
        while not rospy.is_shutdown():
            for h in range(H):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = h/D
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            for y in range(20):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break


    
    # x, y is the x, y distance relative to itself
    # z is absolute z distance
    # TODO: solve 0
    def tracking (self, x0, y0, z0):
        start = rospy.get_time()
        t = 0.0; t_final = 30.0; dt = 0.05 	
	while not rospy.is_shutdown():
	      while t < t_final:
	            self.msg_position.x = 0.0
	            self.msg_position.y = 0.0
	            self.msg_position.yaw = 0.0
	            self.msg_position.z = z0
	            now = rospy.get_time()
	            if (now - start > t_final):
	               break
	    	    #
	    	    t = t + dt
	            self.pub_position.publish(self.msg_position)
	            self.rate.sleep()


    
    """
    # land from last zDistance
    def land (self):
        # get last height
        zDistance = self.msg.zDistance

        while not rospy.is_shutdown():
            while zDistance > 0:
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
                zDistance -= 0.2
        self.stop_pub.publish(self.stop_msg)
    """
    
    

def handler(cf):
    H = 50       # resolution of take-off 
    D = 50.0
    cf.takeOff(H)
    cf.tracking(0.0, 0.0, H/D)
    # cf.land()

if __name__ == '__main__':
    rospy.init_node('multi_cf_node', anonymous=True)

    cf1 = Crazyflie("cf1")
    cf2 = Crazyflie("cf2")
    cf3 = Crazyflie("cf3")
    cf4 = Crazyflie("cf4")
    cf5 = Crazyflie("cf5")

    t1 = Thread(target=handler, args=(cf1,))
    t2 = Thread(target=handler, args=(cf2,))
    t3 = Thread(target=handler, args=(cf3,))
    t4 = Thread(target=handler, args=(cf4,))
    t5 = Thread(target=handler, args=(cf5,))

    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()



