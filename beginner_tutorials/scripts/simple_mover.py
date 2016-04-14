#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class simple_mover:
    x = 1
    az = 0
    def __init__(self):
        self.sm()
    def processScan(self, scan):
        i = 0
        obj_in_front = false
        for ray in scan.ranges:
            ang = scan.angle_min+i*scan.angle_increment
            zed = scan.angle_min +scan.angle_max
            if(ang > zed - 20*scan.angle_increment and ang < zed + 20*scan.angle_increment):
                if(ray < 5):
                    self.x = 0
                    self.az = .5
                    obj_in_front = true
            i+=1
        if(obj_in_front == false):
            self.x = 1
            self.az = 0


    def sm(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('base_sensor_1', LaserScan, self.processScan)
        rospy.init_node('simple_mover', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
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
        s = simple_mover()
    except rospy.ROSInterruptException:
        pass