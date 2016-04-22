#!/usr/bin/env python
# author: gauthampdas (pdasgautham-at-yahoo-dot-com)
# isrc, university of ulster, derry, uk

import roslib; roslib.load_manifest('pioneerControl')
import rospy
import tf
import nav_msgs.msg
import p2os_driver.msg
import geometry_msgs.msg
class commandAndGo:
	x = 0
	az = 0
	startPose = None
	currPose = None
	def __init__(self, t, val, r):
		ns = "pioneer"
		sonSub = rospy.Subscriber(ns+"/sonar", p2os_driver.msg.SonarArray, processSonar)
		odoSub = rospy.Subscriber(ns+"/pose", nav_msgs.msg.Odometry, processOdometry)

		## publishers
		velPub = rospy.Publisher(ns+"/cmd_vel", geometry_msgs.msg.Twist)
		motPub = rospy.Publisher(ns+"/cmd_motor_state", p2os_driver.msg.MotorState)

		rate = rospy.Rate(1)
		count = 0
		vel = geometry_msgs.msg.Twist()
		if(t != 'a'):
			vel.linear.x = r
			vel.angular.z = 0
		else:
			vel.linear.x = 0
			vel.angular.z = r

		while not rospy.is_shutdown() and self.checkEnd(t, val):
			velPub.publish(vel)
			rate.sleep()


	def checkEnd(self, t, val):
		if(t == 'a'):
			if(self.getAng() >= val):
				return False
			return True
		else:
			if(self.getDist() >=val):
				return False
			return True

	def processOdometry(self, odoMsg):
		if self.startPose == None:
			self.startPose = odoMsg.pose
		else:
			self.currPose = odoMsg.pose

	def getDist(self):
		s = [self.startPose.pose.position.x, self.startPose.pose.position.y, self.startPose.pose.position.z]
		c = [self.currPose.pose.position.x, self.currPose.pose.position.y, self.currPose.pose.position.z]
		dist = sqrt((s[0]-c[0])**2+(s[1]-c[1])**2+(s[0]-c[0])**2)
		return dist

	def getAng(self):
		s = (self.startPose.pose.orientation.x, self.startPose.pose.orientation.y, self.startPose.pose.orientation.z, self.startPose.pose.orientation.q)
		c = (self.currPose.pose.orientation.x, self.currPose.pose.orientation.y, self.currPose.pose.orientation.z, self.currPose.pose.orientation.q)
		s = tf.transformations.euler_from_quaternion(s)
		c = tf.transformations.euler_from_quaternion(c)
		ang = abs(s[2] - c[2])
		return ang

	def processSonar(self, sonMsg, sonLock):
	  	print sonMsg.ranges 

if __name__ == "__main__":
	while True:
		t, end, speed = raw_input("Enter Move Type, End dist, speed:  ").split()
		end = int(end)
		speed = int(speed)
		commandAndGo(t, end, speed)
		print("Finished\n")




