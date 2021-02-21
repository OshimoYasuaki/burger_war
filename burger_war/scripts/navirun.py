#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def start(self):
	global status
        r = rospy.Rate(10) # change speed 5fps
        self.setGoal(-0.8,0,0) #a到着
        self.setGoal(-0.8,0,3.14)
        self.setGoal(-0.8,0,0)
        self.setGoal(-0.5,0,0)#d到着
        self.setGoal(-0.5,0,3.1415/4)
        status = 2
    
    def strategy(self):
	global status
	while True:
	    if status == 1:
                self.setGoal(-0.5,0,3.1415)#d到着
                self.setGoal(-0.8,0,3.1415)#a到着
		self.setGoal(-0.8,0,-3.14/2)
                self.setGoal(-0.8,-0.5,-3.14/2)#b到着
                self.setGoal(-0.8,-0.5,0) #マーカ見る
       	        self.setGoal(-0.8,-0.5,3.1415/2)
	        self.setGoal(-0.8,0,3.1415/2)
	        self.setGoal(-0.8,0.5,3.1415/2)#c到着
                self.setGoal(-0.8,0.5,0)#マーカ見る
	        self.setGoal(-0.8,0.5,-3.1415/2)
	        self.setGoal(-0.8,0,-3.1415/2) #a到着
	        self.setGoal(-0.8,0,0)
	        self.setGoal(-0.5,0,0) #d到着,マーカ見る
                self.setGoal(-0.5,0,3.14/4)
                status = 2
            elif status == 2:
                self.setGoal(-0,0.5,3.14/4)#e到着
                self.setGoal(0,0.5,0) #マーカ見る
                self.setGoal(0,0.5,3.1415)#マーカ見る
	        self.setGoal(0,0.5,-3.1415/2) #マーカ見る
                self.setGoal(0,0.5,-3.1415/4)
	        status = 3
	    elif status == 3:
                self.setGoal(0.5,0,-3.1415/4) #f到着
	        self.setGoal(0.5,0,0)
                self.setGoal(0.8,0,0) #g到着
                self.setGoal(0.8,0,3.1415/2)
	        self.setGoal(0.8,0.5,3.1415/2)#h到着
                self.setGoal(0.8,0.5,3.1415)#マーカを見る
	        self.setGoal(0.8,0.5,-3.1415/2)
	        self.setGoal(0.8,0,-3.1415/2)#g到着
	        self.setGoal(0.8,-0.5,-3.1415/2)#i到着
	        self.setGoal(0.8,-0.5,3.1415)#マーカを見る
	        self.setGoal(0.8,-0.5,3.1415/2)
	        self.setGoal(0.8,0,3.1415/2)#g到着
	        self.setGoal(0.8,0,3.1415)
	        self.setGoal(0.5,0,3.1415)#f到着、マーカを見る
                self.setGoal(0.5,0,-3.1415/4*3)
	        status = 4
            elif status == 4:
                self.setGoal(0,-0.5,-3.1415/4*3) #j到着
		self.setGoal(0,-0.5,0)#マーカを見る
	        self.setGoal(0,-0.5,3.1415/2)#マーカを見る
	        self.setGoal(0,-0.5,3.1415/2)#マーカを見る
	        self.setGoal(0,-0.5,3.1415/4*3)
	        status = 1



if __name__ == '__main__':
    status = 0
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.start()
    bot.strategy()

