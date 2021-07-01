#!/usr/bin/python3

import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32

import math
import numpy as np
# from sklearn.cluster import DBSCAN
import math
import time
class LegL_SLAM():
	def __init__(self):
		self.rospy = rospy
		self.rospy.init_node('formationControl', anonymous = True)
		self.rospy.loginfo("Starting Formation Control")
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initVariables()
		self.mainControl()
		return

	def initParameters(self):
		self.move_base_vel_topic = self.rospy.get_param("~move_base/vel_topic","/cloudwalker/cmd_vel")
		self.legLaserTopic = self.rospy.get_param("~legLaser_topic","/distance")        
		self.velTopic = self.rospy.get_param("~cloudwalker/vel_topic","/cmd_vel")
		self.controlRate = self.rospy.get_param("~control_rate", 20.0)
		self.dist_Thresh = self.rospy.get_param("~dist_Thresh", 0.5)
		return

	def initPublishers(self):
		self.pubVel = self.rospy.Publisher(self.velTopic, Twist, queue_size = 10)
		return

	def initSubscribers(self):
		self.distanceMean = self.rospy.Subscriber(self.legLaserTopic, Float32, self.callbackPos)
		self.sub_movebase_cmd = self.rospy.Subscriber(self.move_base_vel_topic, Twist , self.callback_movebase)
		# self.subPosHead = self.rospy.Subscriber(self.posHeadTopic, String, self.callbackPosHead)

		return

	def initVariables(self):
		self.rate = self.rospy.Rate(self.controlRate)
		self.move_base_cmd_msg=Twist()
		self.change = False
		self.mediaX = 0.0
		self.users_activity_range=0.2
		return
	
	def callback_movebase(self,msg):
		self.move_base_cmd_msg=msg
		return 

	def callbackPos(self, msg):

		self.mediaX = msg.data
		self.change = True
	
	def allow_move_base(self):
		if self.mediaX > self.dist_Thresh:
			self.move_base_cmd_msg.linear.x=0*self.move_base_cmd_msg.linear.x
			self.move_base_cmd_msg.angular.z=0*self.move_base_cmd_msg.angular.z
		else:
			base_distance = self.dist_Thresh-self.mediaX
			if(base_distance>0.0 and base_distance <=0.2):
				proportional_gain = (1/self.users_activity_range)*base_distance	
				self.move_base_cmd_msg.linear.x=proportional_gain*self.move_base_cmd_msg.linear.x
				self.move_base_cmd_msg.angular.z=proportional_gain*self.move_base_cmd_msg.angular.z
		return

	def mainControl(self):
		while not self.rospy.is_shutdown():
			self.msg = Float32()
			# Se o Laser nao identifica, ele retorna 0
			if self.change:	
				# Se o Laser identificou alguma coisa	
				self.allow_move_base()
				self.pubVel.publish(self.move_base_cmd_msg)
				self.change = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		legL = LegL_SLAM()
	except rospy.ROSInterruptException:
		pass
