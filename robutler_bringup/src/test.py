#! /usr/bin/python3


import rospy
from menu import moveToPosition, makeTextMarker

rospy.init_node("move")


# Move to coordinates
x = -2.5
y = -4.5
r = 1.0
moveToPosition(x, y, r)



# Text doesn't work - error with using a global variable?

# server = InteractiveMarkerServer("move")
# makeTextMarker(text="Hello!", color=[0.3, 0.3, 0.9])
# rospy.spin()
