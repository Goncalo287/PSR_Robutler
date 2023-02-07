#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios

WAFFLE_MAX_LIN_VEL = 1.5
WAFFLE_MAX_ANG_VEL = 2.0

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.15

msg = """
\n\n\n\n\n\n\n
---------------------------
Controls:

   q    w

   a    s    d


 wasd: move
space: force stop
    q: reset angular velocity

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "Velocity:\n  Linear: %s\n  Angular: %s " % (round(target_linear_vel, 2), round(target_angular_vel,2 ))

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robutler_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    key_pressed = True
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                key_pressed = True
                
            elif key == 's' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                key_pressed = True
                
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                key_pressed = True
                
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                key_pressed = True
                
            elif key == ' ':
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                key_pressed = True
                
            elif key == 'q':
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                key_pressed = True

            else:
                if (key == '\x03'):
                    break

            if key_pressed:
                print(msg)
                key_pressed = False
                print(vels(target_linear_vel,target_angular_vel))

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except Exception as ex:
        print(e)
        print(ex)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
