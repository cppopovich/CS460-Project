#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class wall_follow:
    startTime = time.time()
    x = 1
    az = 0
    moveType = 0
    def __init__(self):
        self.sm()

    #Get a y distance, biased by the angle that the ray is cast. 
    #Rays far from 90 should return larger numbers, so they will be
    #less able to trip the wall Found condition
    def getY(self, ranges):
        return [y*(2-math.sqrt(math.sqrt(math.sin((i+1)*math.pi/180)))) for i, y in enumerate(ranges)]
    #Get a x distance, biased by the angle that the ray is cast
    #Rays close to 0 should be larger, so they will be
    #more able to trip the right wall missing condition
    def sweepRight(self, ranges):
        return [x*(math.cos((i)*math.pi/180)) for i, x in enumerate(ranges)]
    

    '''
    ##WaggleFollow algorithem
    ##1. Start by driving until a wall is found
    ##2. If wall is directly in front and within panicDist, stop and turn left
    ##3. else If wall is direcly in front and within linDist, slow down and turn left
    ##4. else If the distance on the right of the robot is greater than turnDist, turn right
    '''
    def waggleFollow(self, scan):
        ##High turnSpeed - Likely faster than the bot can actually go, but we want it to 
        ##turn as fast a possible
        turnSpeed = 359*math.pi/180.0
        goSpeed = 1

        ##Magic Numbers : found through trial and error
        linDist = .44
        panicDist = .42
        turnDist = .31

        #Start State
        #drive straight until wall is hit
        if self.moveType == 0:
            if all(d > linDist for d in self.getY(scan.ranges)):
                self.x = 1
                self.az = 0
            else:
                self.x = 0
                self.moveType = 1

        #Algorithm state
        #turn logic - if there is a wall in front, turn left. 
        #If there is no wall on the right, turn right. Else just go straight
        elif self.moveType == 1:
            self.x = goSpeed
            #Panic distance - stop before you hit the wall
            if not all(d > panicDist for d in self.getY(scan.ranges)):
                self.az = turnSpeed
                self.x = 0
            #linDist - the values chosen cause occilation, by not stopping while turning,
            #we can waggle down straightways faster
            elif not all(d > linDist for d in self.getY(scan.ranges)):
                self.az = turnSpeed
                self.x = goSpeed/2.0
            #Check for openings
            elif any(i >= turnDist for i in self.sweepRight(scan.ranges)[0:91]):
                self.az = -turnSpeed
            #None apply: go straight
            else:
                self.az = 0

    def sm(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('base_scan_1', LaserScan, self.waggleFollow)
        rospy.init_node('wall_follow', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown(): # and time.time() - self.startTime < 5*60:
            twist = Twist()
            twist.linear.x = self.x
            twist.angular.z = self.az
            pub.publish(twist)

            rate.sleep()
            
        rospy.loginfo("Stopping Bot")
        twist = Twist()
        pub.publish(twist)

if __name__ == '__main__':
    try:
        s = wall_follow()
    except rospy.ROSInterruptException:
        pass