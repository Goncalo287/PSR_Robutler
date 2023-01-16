#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebaseClient(x, y, r):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()


    # Position (xyz)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # Rotation (xyzw)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = r
    goal.target_pose.pose.orientation.w = 1.0


    # Sends the goal to the action server.
    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()   


if __name__ == "__main__":

    x = -1.5
    y = -4.0
    r = 3.0

    try:
        rospy.init_node('movebase_client_py')
        result = movebaseClient(x, y, r)
        if result:
            rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
