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

import tkinter as tk
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry


# Global variables
server = None
pose = None


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


def moveToPosition(x, y, r, wait=False):
    """
    Move robot to 2D position (x, y) with rotation (r) in radians
    Wait (bool): if the robot should delay other actions until it reaches the goal (False by default)
    """
    print("moving to position... {}, {}, {}".format(x, y, r))

    # Init
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Position (xyz) & Orientation (xyzw)
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = float(r)
    goal.target_pose.pose.orientation.w = 1.0

    # Send goal to the action server
    client.send_goal(goal, done_cb=goalDoneCallback)

    if wait:
        client.wait_for_result()


def moveCallback( _ , goal, goal_dict, wait=False):
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
    else:
        valid_coords = False

    # If coordinates are valid, move to goal
    if valid_coords:
        print("New goal: " + goal)
        moveToPosition(goal_x, goal_y, goal_r, wait)
        makeTextMarker( text = "Moving to \"{}\"".format(goal),
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
    makeTextMarker(text = "Ready")


def coordinatesCallback( _ ):


    root = tk.Tk()
    root.title("Enter Coordinates")

    x_label = tk.Label(root, text="X:")
    x_label.grid(row=0, column=0, padx=10, pady=10)

    x_entry = tk.Entry(root)
    x_entry.grid(row=0, column=1, padx=10, pady=10)

    y_label = tk.Label(root, text="Y:")
    y_label.grid(row=1, column=0, padx=10, pady=10)

    y_entry = tk.Entry(root)
    y_entry.grid(row=1, column=1, padx=10, pady=10)

    r_label = tk.Label(root, text="R (rad):")
    r_label.grid(row=2, column=0, padx=10, pady=10)

    r_entry = tk.Entry(root)
    r_entry.grid(row=2, column=1, padx=10, pady=10)

    def submit():
        x_raw = x_entry.get()
        y_raw = y_entry.get()
        r_raw = r_entry.get()
        try:
            x=float(x_raw)
            y=float(y_raw)
            r=float(r_raw)

            if -10 < x < 10 and -10 < y < 10:
                valid_input = True
            else:
                valid_input = False
        except ValueError:
            valid_input = False

        if not valid_input:
            makeTextMarker( text = "Invalid input!",
                            color = [0.8, 0.2, 0.2] )
            return

        makeTextMarker( text = "Moving to...\n{} / {} / {} rad".format(round(x,2), round(y,2), r),
                        color = [0.3, 0.8, 0.3] )

        moveToPosition(x, y, r)

        # coordinate_publisher = rospy.Publisher("/coordinates", PoseStamped, queue_size=10)
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = "map"
        # goal_pose.pose.position.x = float(x)
        # goal_pose.pose.position.y = float(y)
        # goal_pose.pose.orientation.z = float(r)
        # coordinate_publisher.publish(goal_pose)
        root.destroy()


    submit_button = tk.Button(root, text="Submit", command=submit)
    submit_button.grid(row=3, column=1, pady=10)

    root.mainloop()


def spawnObjectCallback( _ , model_name):
    """
    Spawn input model on a random location
    """

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('psr_apartment_description') + '/description/models/'

    # Get a random model_placement
    if model_name == "sphere_v" or "sphere_r":

        placements = []
        placements.append({'pose':Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                    'room':'large_bedroom', 'place': 'bed'})
        placements.append({'pose':Pose(position=Point(x=-7.33, y=5.29, z=0.58), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                    'room':'large_bedroom', 'place': 'bedside_cabinet'})
        model_placement = random.choice(placements)

    elif model_name == "laptop":
        print("laptop")
        return # Replace with code to get random model_placement

    elif model_name == "bottle":
        print("bottle")
        return # Replace with code to get random model_placement

    elif model_name == "person":
        print("person")
        return # Replace with code to get random model_placement

    else:
        print("invalid model name")
        return

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
    spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")


def searchObject(model_name):
    """
    Adapt code from camera.py
    Look for "model_name" at curent position
    Spin 360 degrees while searching for object
    """

    if model_name == "sphere_violet":
        pass

    elif model_name == "sphere_red":
        pass


def searchCallback( _ , model_name, location, goal_dict = {}):
    """
    Search for "model_name" on "location"
        location == 0: "Here"
        location == 1: "Everywhere"
        location == str: goal from saved_locations
    """

    # If location is string -> it's a dictionary key -> move to goal
    if isinstance(location, str):
        moveCallback(0, goal=location, goal_dict=goal_dict, wait=True)
        searchObject(model_name)

    elif location == 0:
        searchObject(model_name)

    # Everywhere:
    elif location == 1:
        self_x = pose.position.x
        self_y = pose.position.y
        print(self_x, self_y)

        # Find nearest point in saved_locations
        # Move there
        # Search
        # Move to next point in list
        # Search
        # Repeat until stopped

        # moveToPosition(1, 2, 3, wait=True)


def initMenu(menu_handler, goals_file):
    """
    Create a menu with a list of goals from a json file
    """

    # Get locations dictionary from json file
    try:
        file_path = os.path.realpath(os.path.dirname(__file__)) + "/" + goals_file
        goal_dict = readJsonFile(file_path)
    except FileNotFoundError:
        exit("Error: " + goals_file + " not found")

    # Move to... coordinates/saved_location
    move_tab = menu_handler.insert( "Move to..." )
    menu_handler.insert("Go to coordinates...", parent=move_tab, callback=coordinatesCallback)

    for goal in goal_dict.keys():
        menu_handler.insert(goal, parent=move_tab, callback=partial(moveCallback, goal = goal, goal_dict = goal_dict))

    menu_handler.insert( "Stop", callback=stopCallback)

    # Spawn Object...
    object_list = ["sphere_violet", "sphere_red", "laptop", "bottle", "person"]
    spawn_tab = menu_handler.insert( "Spawn Object..." )
    for object_name in object_list:
        menu_handler.insert(object_name, parent=spawn_tab, callback=partial(spawnObjectCallback, model_name = object_name))

    # Look for Object...
    search_tab = menu_handler.insert( "Look for Object..." )
    for object_name in object_list:

        search_tab_2 = menu_handler.insert(object_name, parent=search_tab)
        menu_handler.insert("Here", parent=search_tab_2, callback=partial(searchCallback, model_name = object_name, location = 0))
        search_tab_3 = menu_handler.insert("Room...", parent=search_tab_2)
    
        for goal in goal_dict.keys():
            menu_handler.insert(goal, parent=search_tab_3, callback=partial(searchCallback, model_name = object_name, location = goal, goal_dict = goal_dict))

        menu_handler.insert("Everywhere", parent=search_tab_2, callback=partial(searchCallback, model_name = object_name, location = 1))

    # Other...
    other_tab = menu_handler.insert( "Other..." )

    menu_handler.insert( "Spin", parent=other_tab, callback=spinCallback)


def positionCallback(msg):
    # Always have a global variable with the current position
    global pose
    pose = msg.pose.pose


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
    makeTextMarker(text = "Ready")
    server.applyChanges()

    rospy.Subscriber('/odom', Odometry, positionCallback)
    
    # Spin until ctrl+c
    rospy.spin()


if __name__ == "__main__":
    main()