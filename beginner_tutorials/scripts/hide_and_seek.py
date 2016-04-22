#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

'''
Slightly Modified 2016
Christian 
William
Chris
Anupriya
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import roslib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from apriltags_ros.msg import *

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        #class variables
        self.tags_to_visit = []
        self.tags_to_visit_ids = []
        self.tags_visited = []
        self.explore_locs = [{'x': 0, 'y' : 0}, 
        {'x': 3.75, 'y' : 0}, 
        {'x': 6.95, 'y' : 0}, 
        {'x': 6.55, 'y' : -2.15}, 
        {'x': 3.5, 'y' : -2}, 
        {'x': .4, 'y' : -1.5}, 
        {'x': 0, 'y' : -3.5}, 
        {'x': 3.35, 'y' : -3.5}, 
        {'x': 6.55, 'y' : -3.7}]

        self.tfListen = tf.TransformListener()

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
    
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat[0], quat[1], quat[2], quat[3]))

    # Start moving
        self.move_base.send_goal(goal)

    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def search_for_tags(self, pos):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        madeIt = self.goto(pos, quat)
        if(not madeIt):
            return -1
        '''for i in range(4):
            quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
            madeIt = self.goto(pos, quat)
            if(not madeIt):
                return -1'''
        return 0
    def goto_tag(self, pos, tagID):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        madeIt = self.goto(pos, quat)
        if(madeIt):
            self.tags_visited.append(tagID)
            return tagID
        return -2

    def explore(self):
        failures = [[], []]
        print("Starting Exploration")
        while(len(self.tags_to_visit) > 0 or len(self.explore_locs) > 0):
            nextLoc = ""
            if(len(self.tags_to_visit) > 0):
                nextLoc = self.tags_to_visit.pop(0)
                print("go to tag at {} {}".format(nextLoc[1], nextLoc[1]))
                success = self.goto_tag(nextLoc[1], nextLoc[0])
            else:
                nextLoc = self.explore_locs.pop(0)
                print("go to point at {}".format(nextLoc))
                success = self.search_for_tags(nextLoc)
            if(success < 0):
                failures[abs(success) - 1].append(nextLoc)
            elif(success > 0):
                print("Found tag {}\n".format(success))
                if success!=0:
                    self.tags_visited.append(success)
            rospy.sleep(1)

        return failures


    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.getNewTags)

    def getNewTags(self, data):
        for detect in data.detections:
            if detect.id in self.tags_to_visit_ids or detect.id in self.tags_visited:
                continue
            
            self.tfListen.waitForTransform(detect.pose.header.frame_id, "/map", detect.pose.header.stamp, rospy.Duration(1))
            

            transd_pose = self.tfListen.transformPose('/map', detect.pose)
            outpos = {'x': transd_pose.pose.position.x, 'y':transd_pose.pose.position.y}
            self.tags_to_visit.append([detect.id, outpos])
            self.tags_to_visit_ids.append(detect.id)
            rospy.logwarn("try success")

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        navigator.listener()
        failedLocs = navigator.explore()
        '''if(len(failedLocs[0]) > 0 or len(failedLocs[1]) > 0):
            navigator.explore_locs = failedLocs[0]
            navigator.tags_to_visit = failedLocs[1]
            navigator.explore()'''

        '''
        # Input pixels, convert to map coordinates
        pX = int(input('Enter the x coordinate: '))
        pY = int(input('Enter the y coordinate: '))
        x = -100 + 0.05 * pX
        y = -100 + 0.05 * (4000 - pY)
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")
        '''

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


