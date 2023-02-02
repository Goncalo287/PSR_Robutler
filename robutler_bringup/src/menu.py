#!/usr/bin/env python3

import os
import rospy
import json
from functools import partial

# Menus
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

# Move to goal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Global variables
server = None


def readJsonFile(path):
    """
    Input: path to json file
    Return: dictionary with contents of the file
    """
    try:
        with open(path, 'r') as file:
            contents = json.load(file)
        return contents
    except Exception:
        print("Invalid json file: " + path)
        return None



def makeMenuMarker():
    """
    Create a box that follows the robot
    """
    
    # Create cube marker
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.1


    # Create interactive marker with cube and insert it on server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.scale = 1
    int_marker.name = "menu_marker"

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)

    server.insert(int_marker)
    server.applyChanges()



def makeTextMarker(text = "Unknown", color = [0.5, 0.5, 0.5]):
    """
    Display text above the robot
    Inputs: text (str)
            color [r, g, b]
    """

    # Create text marker
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING

    # Text properties
    marker.text = text
    marker.scale.z = 0.37
    marker.pose.position.z = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1

    # Create interactive marker with text and insert it on server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "text_marker"

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)

    server.insert(int_marker)
    server.applyChanges()



def goalDoneCallback(state, result):
    """
    Called when a goal is "done":
        If it was canceled, state == 2
        If it was reached, state == 3
    """

    if state == 3:
        print("Goal done: reached")
        makeTextMarker( text = "Goal reached",
                        color = [0.3, 0.8, 0.3])
    else:
        print("Goal done: canceled")



def moveToPosition(x, y, r):
    """
    Move robot to 2D position (x, y) with rotation (r) in radians
    """

    # Init
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Position (xyz) & Orientation (xyzw)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = r
    goal.target_pose.pose.orientation.w = 1.0

    # Send goal to the action server
    client.send_goal(goal, done_cb=goalDoneCallback)



def moveCallback( _ , goal, goal_dict):
    """
    Called when the user clicks a goal from the list. Move robot to that goal
    Inputs: goal: string, a key of goal_dict
            goal_dict: dictionary with goals and their coordinates    
    """

    # Get goal coordinates from dictionary
    if isinstance(goal, str):
        try:
            goal = goal.lower()
            goal_x = goal_dict[goal]["x"]
            goal_y = goal_dict[goal]["y"]
            goal_r = goal_dict[goal]["r"]

            if isinstance(goal_x, float) and isinstance(goal_y, float) and isinstance(goal_r, float):
                valid_coords = True
            else:
                valid_coords = False
        except KeyError:
            valid_coords = False

    # If coordinates are valid, move to goal
    if valid_coords:
        print("New goal: " + goal)
        moveToPosition(goal_x, goal_y, goal_r)
        makeTextMarker( text = "Moving to \"{}\"...".format(goal),
                        color = [0.3, 0.8, 0.3])

    # If goal or coordinares are invalid, display error message
    else:
        print("Invalid goal: " + goal)
        makeTextMarker( text = "Invalid goal",
                        color = [0.8, 0.2, 0.2])



def stopCallback( _ ):
    """
    Called when the user clicks the "Stop" button. Cancel goal
    """

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()
    makeTextMarker( text = "Goal canceled",
                    color = [0.8, 0.2, 0.2])



def initMenu(menu_handler, goals_file):
    """
    Create a menu with a list of goals from a json file
    """

    # Get dictionary from json file
    try:
        file_path = os.path.realpath(os.path.dirname(__file__)) + "/" + goals_file
        goal_dict = readJsonFile(file_path)
    except FileNotFoundError:
        exit("Error: " + goals_file + " not found")


    # Create menu
    move_tab = menu_handler.insert( "New goal..." )

    for goal in goal_dict.keys():
        menu_handler.insert(goal, parent=move_tab, callback=partial(moveCallback, goal = goal, goal_dict = goal_dict))

    menu_handler.insert( "Stop", callback=stopCallback)



def main():

    # Initialize
    rospy.init_node("menu")
    global server
    server = InteractiveMarkerServer("menu")
    menu_handler = MenuHandler()

    # Create menu
    initMenu(menu_handler, goals_file="saved_locations.json")
    makeMenuMarker()
    menu_handler.apply(server, "menu_marker")
    server.applyChanges()

    # Spin until ctrl+c
    rospy.spin()


if __name__ == "__main__":
    main()