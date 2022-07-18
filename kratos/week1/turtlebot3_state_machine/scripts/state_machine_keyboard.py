#!/usr/bin/env python

import rospy
from turtlebot3_state_machine.srv import StateMachineControl

# https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_state_machine_keyboard')
    rospy.wait_for_service('state_machine_control')
    state_machine_control = rospy.ServiceProxy('state_machine_control', StateMachineControl)
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            state_machine_control('move',1)
        elif key == 'x':
            state_machine_control('move',-1)
        elif key == 'a':
            state_machine_control('turn',1)
        elif key == 'd':
            state_machine_control('turn',-1)
        elif key == 's':
            state_machine_control('stop',1)
        elif key == 'q' or key == '\x03':
            break
    