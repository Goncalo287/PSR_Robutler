#! /usr/bin/python3


import rospy
from menu import moveToPosition

rospy.init_node("move")
moveToPosition(-2, 1, 0)


