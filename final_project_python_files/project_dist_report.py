#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

#Collects odometry data and reports distance to target
class ReportGenerator:

    def tCallback(self,data):
        self.t_loc = data.pose.pose.position

    def h0Callback(self,data):
        self.h0_loc = data.pose.pose.position

    def h1Callback(self,data):
        self.h1_loc = data.pose.pose.position

    #Computes the distance for each hunter
    #Based on last reported location
    def computeDistances(self):
        def dist(a,b):
            return math.sqrt((a.x-b.x)**2+(a.y-b.y)**2)
        return (dist(self.t_loc,self.h0_loc), dist(self.t_loc,self.h1_loc))

    #Main report thread
    #every X seconds, uses distance information
    def report(self):
        #initializing locations to None
        self.t_loc = None
        self.h0_loc = None
        self.h1_loc = None

        #setup for publishers and subscribers
        self.pub = rospy.Publisher('dist_rep', String, queue_size=1)
        rospy.init_node('Report', anonymous=True)
        self.sub = rospy.Subscriber('robot_2/odom', Odometry, self.tCallback)
        self.sub = rospy.Subscriber('robot_0/odom', Odometry, self.h0Callback)
        self.sub = rospy.Subscriber('robot_1/odom', Odometry, self.h1Callback)

        #periodically computes and publishes distances
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if None in [self.t_loc,self.h0_loc,self.h1_loc]:
                continue
            (dist_0,dist_1) = self.computeDistances()
            dist_msg = str(dist_0)+" "+str(dist_1)
            self.pub.publish(dist_msg)

if __name__ == '__main__':
    try:
        rg = ReportGenerator()
        rg.report()
    except rospy.ROSInterruptException:
        pass
