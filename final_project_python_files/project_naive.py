#!/usr/bin/env python

import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
#TODO requires move_base (actionlib?)
####

#VARIABLES
#distance to target
#   updated by subscriber/callback
#expected location of target
#   Choose one of the two possible locations,
#   then update when new information comes.
#location of each hunter
####

#Moves directly towards target.
#Initially may guess where target is.
class NaiveHunterAI:

    def __init__():
        self.t_loc = Point(0,0,0)
        self.h0_loc = None
        self.h1_loc = None
        self.h0_sub = rospy.Subscriber('h0/odom', Odometry, h0Callback)
        self.h1_sub = rospy.Subscriber('h1/odom', Odometry, h1Callback)
        self.d_sub = rospy.Subscriber('dist_rep', Odometry, distCallback)

    def distCallback(data):
        dList = data.data.split()
        self.d0 = float(dList[0])
        self.d1 = float(dList[1])
        (p1,p2) = self.triangulate()
        #Choose point closest to previous target location
        if distance(p1,self.t_loc) < distance(p2,self.t_loc):
            self.t_loc = p1
        else:
            self.t_loc = p2
        #TODO move_base to self.t_loc for each hunter

    def h0Callback(data):
        self.h0_loc = data.pose.pose.position

    def h1Callback(data):
        self.h1_loc = data.pose.pose.position

    def triangulate():
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
