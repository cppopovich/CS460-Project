#!/usr/bin/env python

import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

#Moves directly towards target.
#Initially may guess where target is.
class NaiveHunterAI:

    def __init__(self):
        self.t_loc = Point(0,0,0)
        self.h0_loc = None
        self.h1_loc = None
        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.h0_sub = rospy.Subscriber('h0/odom', Odometry, self.h0Callback)
        self.h1_sub = rospy.Subscriber('h1/odom', Odometry, self.h1Callback)
        self.d_sub = rospy.Subscriber('dist_rep', Odometry, self.distCallback)

    def distCallback(self,data):
        dList = data.data.split()
        self.d0 = float(dList[0])
        self.d1 = float(dList[1])
        (p1,p2) = self.triangulate()
        #Choose point closest to previous target location
        if distance(p1,self.t_loc) < distance(p2,self.t_loc):
            self.t_loc = p1
        else:
            self.t_loc = p2
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(self.t_loc, Quaternion())
        # Start moving
        self.move_base.send_goal(goal)

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")

    def h0Callback(self,data):
        self.h0_loc = data.pose.pose.position

    def h1Callback(self,data):
        self.h1_loc = data.pose.pose.position

    def triangulate(self):
        if None in [self.h0_loc,self.h1_loc]:
            return
        (x1,y1) = (self.h0_loc.x,self.h0_loc.y)
        (x2,y2) = (self.h1_loc.x,self.h1_loc.y)
        (d1,d2) = (self.d0,self.d1)
        d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        a = (d1*d1 - d2*d2 + d*d)/(2*d)
        h = math.sqrt(d1*d1 - a*a)
        (x3,y3) = ((x2-x1)*a/d + x1,(y2-y1)*a/d + y1)
        xx1 = x3 + h*(y2-y1)/d
        yy1 = y3 - h*(x2-x1)/d
        xx2 = x3 - h*(y2-y1)/d
        yy2 = y3 + h*(x2-x1)/d
        return ((xx1,yy1),(xx2,yy2))

if __name__ == '__main__':
    try:
        rospy.init_node('', anonymous=False)
        ai = NaiveHunterAI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
