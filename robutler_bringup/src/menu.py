#!/usr/bin/env python3

import os
import rospy
import json
from functools import partial
import math

# Menus
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

# Move to goal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

#spawn objects

import random
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


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
    marker.pose.position.z = 0.75
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

    global server
    if server is None:
        server = InteractiveMarkerServer("menu")
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


def spinCallback( _ ):
    """
    Do a full rotation
    """

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    # Spin
    makeTextMarker( text = "Spinning...",
                    color = [0.2, 0.2, 0.8])

    # Calculate spin duration from speed (rad/s)
    rotation_speed = 0.75
    time_end = rospy.get_rostime().secs + 2*math.pi/rotation_speed

    while rospy.get_rostime().secs < time_end:
        twist.angular.z = rotation_speed
        publisher.publish(twist)

    # Stop spinning
    twist.angular.z = 0
    publisher.publish(twist)
    makeTextMarker(text = "Idle")

def AddCallback( _ ):
    rospy.init_node('insert_object',log_level=rospy.INFO)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('psr_apartment_description') + '/description/models/'


    #Add here mode poses
    placements = []
    placements.append({'pose':Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'large_bedroom', 'place': 'bed'})
    placements.append({'pose':Pose(position=Point(x=-7.33, y=5.29, z=0.58), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'large_bedroom', 'place': 'bedside_cabinet'})


    model_names = ['sphere_v']

    # Add here several models. All should be added to the robutler_description package
    model_name = random.choice(model_names)

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)


    model_placement = random.choice(placements)
    name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
    spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")

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
    menu_handler.insert( "Spin", callback=spinCallback)
    menu_handler.insert( "Add Object", callback=AddCallback)


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
    makeTextMarker(text = "Idle")
    server.applyChanges()

    # Spin until ctrl+c
    rospy.spin()


if __name__ == "__main__":
    main()