#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_state_machine.srv import StateMachineControl

class TurtleBot3_State_Machine:

    def __init__(self):
        self.state = "s"
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.srv = rospy.Service('state_machine_control', StateMachineControl, self.handle)
        self.velocity = Twist()

    def move(self, dir):
        if dir == 1:
            self.state = "w"
            self.velocity = Twist()
            self.velocity.linear.x = 0.5
        elif dir == -1:
            self.state = "x"
            self.velocity = Twist()
            self.velocity.linear.x = -0.5
        else:
            return False
        return True
        

    def turn(self, dir):
        if dir == 1:
            self.state = "a"
            self.velocity = Twist()
            self.velocity.angular.z = 0.5
        elif dir == -1:
            self.state = "d"
            self.velocity = Twist()
            self.velocity.angular.z = -0.5
        else:
            return False
        return True
        

    def stop(self):
        self.state = "s"
        self.velocity = Twist()
        return True

    def publish(self):
        self.pub.publish(self.velocity)

    def handle(self, req):
        success = False
        if req.state == "move":
            success = self.move(req.dir)
        elif req.state == "turn":
            success = self.turn(req.dir)
        elif req.state == "stop":
            success = self.stop()
        if success:
            self.publish()
        return success

if __name__ == '__main__':
    rospy.init_node('turtlebot3_state_machine')
    TurtleBot3_State_Machine()
    rospy.spin()