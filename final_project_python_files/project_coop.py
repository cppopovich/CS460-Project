#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class CoopHunterAI:

    def __init__(self):
        self.t_loc = None
        self.t_loc1 = Point(0,0,0)
        self.t_loc2 = Point(0,0,0)
        self.h0_loc = None
        self.h1_loc = None
        self.state = 1
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
        ###Decide where to move
        if state is 1:
            #Not sure which triangulated point
            # is correct.
            self.t_loc1 = p1
            self.t_loc2 = p2
            if distance(p1,self.t_loc1) < distance(p1,self.t_loc2):
                self.t_loc1 = p1
                self.t_loc2 = p2
            else:
                self.t_loc1 = p2
                self.t_loc2 = p1
            #TODO check if correct location is found
        if state is 2:
            #found correct location
            #Commence surrounding process

            #check for correct triangulated point
            if distance(p1,self.t_loc) < distance(p2,self.t_loc):
                new_loc = p1
            else:
                new_loc = p2
            #test if close enough to move to state 3
            if distance(new_loc,self.t_loc) < min(self.d0,self.d1):
                state = 3
                self.t_loc = new_loc
            else:
                dx = new_loc.x - self.t_loc.x
                dy = new_loc.y - self.t_loc.y
                self.t_loc = new_loc
                self.t_loc1.x = self.t_loc.x + dx
                self.t_loc1.y = self.t_loc.y + dy
                self.t_loc2.x = self.t_loc.x - dx
                self.t_loc2.y = self.t_loc.y - dy
        if state is 3:
            #Close enough to move directly towards target
            if distance(p1,self.t_loc) < distance(p2,self.t_loc):
                self.t_loc = p1
            else:
                self.t_loc = p2
        ##
        #TODO modify to send goals to both robots
        #TODO if state is 1 or 2, move to t_loc1 and t_loc2
        #     if state is 3, move both to t_loc
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(self.t_loc, Quaternion())
        # Start moving
        self.move_base.send_goal(goal)

    def distance(p1,p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

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
        rospy.init_node('coop', anonymous=False)
        ai = CoopHunterAI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
