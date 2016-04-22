#!/usr/bin/env python
# license removed for brevity
import tf
import rospy
import math
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
class simpleTest:

    def __init__(self, t, end, speed):
        #t is the type
        self.t = t
        #end is where its suppose to stop
        self.end = end
        #how fast it is going to move 
        self.speed = speed
        #pose we will pull from the odometry
        self.startPose = None
        self.currPose = None
        #The change in angle
        self.angA = 0
        if t == 'a':
            self.x = 0
            self.az = speed
        else:
            self.x = speed
            self.az = 0
        self.sm()

    def storePose(self, odoMsg):
	if self.t !='a':
        	if self.startPose == None:
            		self.startPose = odoMsg.pose
        	else:
            		self.currPose = odoMsg.pose
	else:
		self.startPose = self.currPose
		self.currPose = odoMsg.pose
		if self.startPose != None:
			self.getAng()
                        

    def getDist(self):
        s = [self.startPose.pose.position.x, self.startPose.pose.position.y, self.startPose.pose.position.z]
        c = [self.currPose.pose.position.x, self.currPose.pose.position.y, self.currPose.pose.position.z]
        dist = math.sqrt((s[0]-c[0])**2+(s[1]-c[1])**2+(s[0]-c[0])**2)
        return dist

    def getAng(self):
        s = (self.startPose.pose.orientation.x, self.startPose.pose.orientation.y, self.startPose.pose.orientation.z, self.startPose.pose.orientation.w)
        c = (self.currPose.pose.orientation.x, self.currPose.pose.orientation.y, self.currPose.pose.orientation.z, self.currPose.pose.orientation.w)
        s = tf.transformations.euler_from_quaternion(s)
        c = tf.transformations.euler_from_quaternion(c)
        #print("{} {}\n".format(c[2], s[2]))
        self.angA +=(abs(abs(c[2]) - abs(s[2])))
        #print("{}".format(self.angA*180/math.pi))

    def checkEnd(self):
        if(self.t == 'a'):
            if((self.angA*(180.0/math.pi))  >= self.end):
                print("{} {}".format(self.angA*180.0/math.pi, self.end))
                return False
            return True
        else:
            if(self.getDist() >=self.end):
                return False
            return True

    def sm(self):
        #Setting the queue to one because we only want the most recent message
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('pose', Odometry, self.storePose)
        rospy.init_node('simpleTest', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while(self.startPose == None or self.currPose == None):
            pass

        while not rospy.is_shutdown() and self.checkEnd():
            twist = Twist()
            twist.linear.x = self.x
            twist.angular.z = self.az
            pub.publish(twist)

            rate.sleep()
            
        rospy.loginfo("Stopping Bot")
        twist = Twist()
        pub.publish(twist)

if __name__ == "__main__":
    while True:
        t, end, speed = raw_input("Enter Move Type, End dist, speed:  ").split()
        end = float(end)
        speed = float(speed)
	if t == 'a':
		speed = speed*math.pi/180.0
        simpleTest(t, end, speed) #.sm() method is invocoted within the constructor
        print("Finished\n")
