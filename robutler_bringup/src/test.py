#! /usr/bin/python3


import rospy
from menu import moveToPosition, makeTextMarker

# Initialize node
rospy.init_node("test")


# Move to coordinates
moveToPosition( x = -2.1, y = -3.5, r = -1.5 )


# Display text
makeTextMarker( text = "Hello!", color = [0.3, 0.3, 0.9] )


# Spin until ctrl+c
rospy.spin()
