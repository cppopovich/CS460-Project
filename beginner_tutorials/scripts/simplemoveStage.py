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
        self.startPoseT = None
        self.currPoseT = None
        #The change in angle
        self.angA = 0
        self.angAT = 0
        if t == 'a':
            self.x = 0
            self.az = speed
        else:
            self.x = speed
            self.az = 0

    def storePose(self, odoMsg):
        if self.t !='a':
            if self.startPose == None:
                self.startPose = odoMsg.pose

            else:
                self.currPose = odoMsg.pose
        else:
            self.startPose = self.currPose
            self.currPose = odoMsg.pose
            if self.startPose != None and self.currPose != None:
                self.getAng()

    def storeTruePose(self, odoMsg):
        if self.t !='a':
            if self.startPoseT == None:
                self.startPoseT = odoMsg.pose
            else:
                self.currPoseT = odoMsg.pose
        else:
            self.startPoseT = self.currPoseT
            self.currPoseT = odoMsg.pose
            if self.startPoseT != None and self.currPoseT != None:
                self.getTrueAng()

                        

    def getDist(self):
        s = [self.startPose.pose.position.x, self.startPose.pose.position.y, self.startPose.pose.position.z]
        c = [self.currPose.pose.position.x, self.currPose.pose.position.y, self.currPose.pose.position.z]
        dist = math.sqrt((s[0]-c[0])**2+(s[1]-c[1])**2+(s[0]-c[0])**2)
        return dist

    def getDistT(self):
        s = [self.startPoseT.pose.position.x, self.startPoseT.pose.position.y, self.startPoseT.pose.position.z]
        c = [self.currPoseT.pose.position.x, self.currPoseT.pose.position.y, self.currPoseT.pose.position.z]
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

    def getTrueAng(self):
        s = (self.startPoseT.pose.orientation.x, self.startPoseT.pose.orientation.y, self.startPoseT.pose.orientation.z, self.startPoseT.pose.orientation.w)
        c = (self.currPoseT.pose.orientation.x, self.currPoseT.pose.orientation.y, self.currPoseT.pose.orientation.z, self.currPoseT.pose.orientation.w)
        s = tf.transformations.euler_from_quaternion(s)
        c = tf.transformations.euler_from_quaternion(c)
        #print("{} {}\n".format(c[2], s[2]))
        self.angAT +=(abs(abs(c[2]) - abs(s[2])))
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

    def writePos(self, f):
        f.write('Data Type: {Type} {Dist} {Speed}\n'.format(Type = self.t, Dist = self.end, Speed = self.speed))
        f.write('Actual Stopping Data :\n')
        f.write('Dist:{}\n'.format(self.getDistT()))
        f.write('Ang:{}\n'.format(self.angAT*180.0/math.pi))

        f.write('Robot Calculated Stopping Data:\n')
        f.write('Dist:{}\n'.format(self.getDist()))
        f.write('Ang:{}\n'.format(self.angA*180.0/math.pi))


    def sm(self, f):
        #Setting the queue to one because we only want the most recent message
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('odom', Odometry, self.storePose)
        sub2 = rospy.Subscriber('base_pose_ground_truth', Odometry, self.storeTruePose)
        rospy.init_node('simpleTest', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while(self.startPose == None or self.currPose == None or self.currPoseT == None or self.startPoseT == None):
            pass

        while not rospy.is_shutdown() and self.checkEnd():
            twist = Twist()
            twist.linear.x = self.x
            twist.angular.z = self.az
            pub.publish(twist)
            rate.sleep()
        with open(f, 'a+') as file:
            self.writePos(file) 
        rospy.loginfo("Stopping Bot")
        twist = Twist()
        pub.publish(twist)

if __name__ == "__main__":
    count = 0
    commands = [
    ['d', '1', '.3'], ['d', '1', '.3'], ['d', '1', '.3'], ['d', '1', '.3'], ['d', '1', '.3'], 
    ['d', '1', '.8'], ['d', '1', '.8'], ['d', '1', '.8'], ['d', '1', '.8'], ['d', '1', '.8'],

    ['d', '5', '.3'], ['d', '5', '.3'], ['d', '5', '.3'], ['d', '5', '.3'], ['d', '5', '.3'], 
    ['d', '5', '.8'], ['d', '5', '.8'], ['d', '5', '.8'], ['d', '5', '.8'], ['d', '5', '.8'],

    ['a', '10',  '30'], ['a', '10',  '30'], ['a', '10',  '30'], ['a', '10',  '30'], ['a', '10',  '30'], 
    ['a', '180', '30'], ['a', '180', '30'], ['a', '180', '30'], ['a', '180', '30'], ['a', '180', '30'],
    ['a', '360', '30'], ['a', '360', '30'], ['a', '360', '30'], ['a', '360', '30'], ['a', '360', '30'],

    ['a', '10',  '120'], ['a', '10',  '120'], ['a', '10',  '120'], ['a', '10',  '120'], ['a', '10',  '120'], 
    ['a', '180', '120'], ['a', '180', '120'], ['a', '180', '120'], ['a', '180', '120'], ['a', '180', '120'],
    ['a', '360', '120'], ['a', '360', '120'], ['a', '360', '120'], ['a', '360', '120'], ['a', '360', '120']]

    fileName = "/home/stranjyr/Documents/University/robotics/homework3Logs/stageLog.dat"

    while count < len(commands):
        t = commands[count][0]
        end = float(commands[count][1])
        speed = float(commands[count][2])
        if t == 'a':
            speed = speed*math.pi/180.0

        s = simpleTest(t, end, speed)
        count+=1
        s.sm(fileName)
