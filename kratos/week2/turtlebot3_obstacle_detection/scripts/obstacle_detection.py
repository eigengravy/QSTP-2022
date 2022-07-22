#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 

class Turtlebot3_Obstacle_Detection:
    def __init__(self):
        self.sub = rospy.Subscriber("/scan", LaserScan, self.handle)
        self.pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)

    def handle(self, data):
        print(data.ranges[0])
        threshold = 2
        if data.ranges[0] > threshold:
            velocity = Twist()
            velocity.linear.x = 0.1
            self.pub.publish(velocity)
        else:
            velocity = Twist()
            self.pub.publish(velocity)
        
if __name__ == "__main__":
    rospy.init_node('obstacle_detection')
    Turtlebot3_Obstacle_Detection()
    rospy.spin()



