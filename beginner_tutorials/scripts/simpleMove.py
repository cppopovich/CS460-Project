#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
class simpleTest:

    def __init__(self, t, end, speed):
        self.t = t
        self.end = end
        self.speed = speed
        self.startPose = None
        self.currPose = None
        self.truePose = None
        if t == 'a':
            self.x = 0
            self.az = speed
        else:
            self.x = speed
            self.az = 0

    def storePose(self, odoMsg):
        if self.startPose == None:
            self.startPose = odoMsg.pose
        else:
            self.currPose = odoMsg.pose

    def storeTruePose(self, odoMsg):
        self.truePose = odoMsg.pose

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
        ang = abs(abs(s[2]) - abs(c[2]))
        return ang

    def checkEnd(self):
        if(self.t == 'a'):
            if(self.getAng() >= self.end):
                return False
            return True
        else:
            if(self.getDist() >=self.end):
                return False
            return True

    def writePos(self, f, state):
        f.write('Odometery {}\n'.format(state))
        f.write("{} {}\n".format(self.currPose.position.x, self.currPose.position.y))
        c = (self.currPose.orientation.x, self.currPose.orientation.y, self.currPose.orientation.z, self.currPose.orientation.q)
        c = tf.transformations.euler_from_quaternion(c)
        self.f.write("{} {} {}\n".format(c[0], c[1], c[2]))

        f.write('True {}\n'.format(state))
        f.write("{} {}\n".format(self.truePose.position.x, self.truePose.position.y))
        c = (self.truePose.orientation.x, self.truePose.orientation.y, self.truePose.orientation.z, self.truePose.orientation.q)
        c = tf.transformations.euler_from_quaternion(c)
        self.f.write("{} {} {}\n".format(c[0], c[1], c[2]))

    def sm(self, file):
        with open(file, 'w') as f:
            f.write('Starting Run\n')
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            sub = rospy.Subscriber('odom', Odometry, self.storePose)
            sub2 = rospy.Subscriber("base_pose_ground_truth", Odometery, self.storeTruePose)
            rospy.init_node('simpleTest', anonymous=True)
            rate = rospy.Rate(10) # 10hz
            while(self.startPose == None and self.currPose == None and self.truePose == None):
                pass
            self.writePos(f, 'Start')
            while not rospy.is_shutdown() and self.checkEnd():
                twist = Twist()
                twist.linear.x = self.x
                twist.angular.z = self.az
                pub.publish(twist)

                rate.sleep()
            
            self.writePos(f, 'End')
            rospy.loginfo("Stopping Bot")
            twist = Twist()
            pub.publish(twist)

if __name__ == "__main__":
    while True:
        t, end, speed = raw_input("Enter Move Type, End dist, speed:  ").split()
        end = int(end)
        speed = int(speed)
        s = simpleTest(t, end, speed)
        fileName = "/home/stranjyr/Documents/University/robotics/homework3Logs/{filename}.dat".format(filename = time.time())
        s.sm(fileName)
        print("Finished\n")