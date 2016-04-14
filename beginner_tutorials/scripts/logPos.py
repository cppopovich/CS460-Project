#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from nav_msgs.msg import Odometry
import tf
from __future__ import with_statement


class wall_follow:
    startTime = time.time()
    x = 1
    az = 0
    moveType = 0
    pose = None
    truePose = None
    f = None
    def __init__(self):
        with open("/home/stranjyr/Documents/University/robotics/homework3Logs/"+str(time.time())+".dat", 'w') as self.f:
            self.sm()

    def storePos(self, od):
        self.pose = od.pose.pose
        self.f.write("{} {}\n".format(self.pose.position.x, self.pose.position.y))
        c = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.q)
        c = tf.transformations.euler_from_quaternion(c)
        self.f.write("{} {} {}\n".format(c[0], c[1], c[2]))

    def sm(self):
        rospy.init_node('scribe', anonymous=True)
        sub = rospy.Subscriber("base_pose_ground_truth", Odometery, self.storeTruePose)
        sub2 = rospy.Subscriber('odom', Odometry, self.storePos)
        rate = rospy.Rate(10) # 10hz
        rospy.spin()

if __name__ == '__main__':
    try:
        s = wall_follow()
    except rospy.ROSInterruptException:
        pass