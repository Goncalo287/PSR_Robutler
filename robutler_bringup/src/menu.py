#!/usr/bin/env python3

import os
import rospy
import json
from functools import partial
import math
import random
import time

# Menus
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


# Move to goal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tkinter as tk

# Spawn objects
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

# Image
from camera import detectSpheres
from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Global variables
server = None
pose = None
bridge = CvBridge()
images = {"camera": None, "object": None, "yolo": None}
labels = []


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
            goal_x = float(goal_dict[goal]["x"])
            goal_y = float(goal_dict[goal]["y"])
            goal_r = float(goal_dict[goal]["r"])
            valid_coords = True

        except KeyError or ValueError:
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


def setSpeedCallback( _ , linear, angular, text):
    """
    Called when the user clicks the "Stop" button. Cancel goal + stop
    """

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular

    time_end = rospy.get_rostime().secs + 0.5
    while rospy.get_rostime().secs < time_end:
        publisher.publish(twist)

    makeTextMarker( text = text,
                    color = [0.2, 0.2, 0.8])


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

        makeTextMarker( text = "Moving to...\n{} / {} / {} rad".format(round(x,1), round(y,1), round(r,1)),
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


def spawnObjectCallback( _ , model_name, object_dict, location=None):
    """
    Spawn input model on a location from spawn_objects.json - if None, random location
    """

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('psr_apartment_description') + '/description/models/'

    # Get placement_data - If no location is specified, get a random one
    placements = object_dict[model_name]

    if location is None:
        placement_data = random.choice(placements)
    else:
        for place in placements:
            if place["name"] == location:
                placement_data = place
                break

    model_pose = Pose(  position=Point( x = placement_data["position"][0],
                                        y = placement_data["position"][1],
                                        z = placement_data["position"][2]),
                        orientation=Quaternion( x = placement_data["orientation"][0],
                                                y = placement_data["orientation"][1],
                                                z = placement_data["orientation"][2],
                                                w = placement_data["orientation"][3]))

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    name = model_name + '_in_' + placement_data["name"]
    spawn_model_prox(name, sdff, model_name, model_pose, "world")


def searchObject(model_name):
    """
    Adapt code from camera.py
    Look for "model_name" at curent position
    Spin 360 degrees while searching for object
    """

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_all_goals()

    makeTextMarker( text = "Looking for \"{}\"".format(model_name),
                    color = [0.2, 0.2, 0.8])

    # Calculate spin duration from speed (rad/s)
    rotation_speed = 0.5
    time_end = rospy.get_rostime().secs + 2*math.pi/rotation_speed

    # Spin while checking camera images
    global images
    global labels
    success = False
    while rospy.get_rostime().secs < time_end:
        rospy.sleep(0.5)
        twist.angular.z = rotation_speed
        publisher.publish(twist)

        # Check for colored spheres
        if model_name in ["sphere_violet", "sphere_red", "all_spheres"]:
            img, success = detectSpheres(images["camera"], model_name)
            if success:
                images["object"] = img
                break

        # Check yolo labels
        elif model_name in ["tvmonitor", "bottle", "person"]:
            if model_name in labels:
                images["object"] = images["yolo"]
                success = True
                break

    # Stop spinning
    twist.angular.z = 0
    publisher.publish(twist)

    if success:
        makeTextMarker( text = "Object found!",
                        color = [0.2, 0.8, 0.2])
    else:
        makeTextMarker( text = "Object not found!",
                        color = [0.8, 0.2, 0.2])
    return success


def searchCallback( _ , model_name, location, goal_dict = {}):
    """
    Search for "model_name" on "location"
        location == 0: "Here"
        location == 1: "Everywhere"
        location == str: goal from saved_locations
    """

    # If location is string -> it's a dictionary key -> move to goal
    if isinstance(location, str):
        makeTextMarker( text = "Looking for \"{}\" in \"{}\"".format(model_name, location),
                        color = [0.2, 0.8, 0.2])
        moveCallback(0, goal=location, goal_dict=goal_dict, wait=True)
        searchObject(model_name)

    # Here
    elif location == 0:
        searchObject(model_name)

    # Everywhere
    elif location == 1:

        # Get robot position
        self_x = float(pose.position.x)
        self_y = float(pose.position.y)

        # Find nearest goal from saved locations
        goal_list = list(goal_dict)
        goal_name = goal_list[0]   # First location by default
        min_distance = 99999

        for goal in goal_dict.keys():
            goal_x = float(goal_dict[goal]["x"])
            goal_y = float(goal_dict[goal]["y"])
            distance = math.sqrt((goal_x-self_x)**2+(goal_y-self_y)**2)

            if distance < min_distance:
                min_distance = distance
                goal_name = goal

        # Make new list of goals starting from the nearest one
        index = goal_list.index(goal_name)
        length = len(goal_list)
        goal_list *= 2
        new_goals = goal_list[index:index+length]
        locations_searched = 0

        # Search in every location
        for goal_name in new_goals:
            locations_searched += 1
            makeTextMarker( text = "Looking for \"{}\"... {} / {}".format(model_name, locations_searched, len(new_goals)),
                        color = [0.2, 0.8, 0.2])
            moveCallback(0, goal=goal_name, goal_dict=goal_dict, wait=True)
            success = searchObject(model_name)
            if success:
                break
        
        if not success:
            makeTextMarker( text = "Object not found!",
                            color = [0.8, 0.2, 0.2])


def removeObjectsCallback( _ , object_dict):

    srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    req = DeleteModelRequest()

    name_list = []

    for obj in object_dict.keys():
        location_list = object_dict[obj]
        for location in location_list:
            name = obj + "_in_" + location["name"]
            name_list += [name]

    success_count = 0
    for name in name_list:
        req.model_name = name
        resp = srv(req)

        if resp.success:
            success_count += 1

    print("Deleted {} objects".format(success_count))


def takePictureCallback( _ ):

    path_this = os.path.realpath(os.path.dirname(__file__))
    time_str = time.strftime("%Y-%m-%d_%H-%M-%S")
    full_path = path_this + "/pictures/" + time_str + ".png"
    success = cv2.imwrite(full_path, images["camera"])
    if success:
        print("Saved image to: " + full_path)
    else:
        print("Failed to save image")


def removeRobotCallback( _ ):
    
    srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    req = DeleteModelRequest()

    req.model_name = "robutler"
    srv(req)


def initMenu(menu_handler):
    """
    Create a menu with a list of goals from a json file
    """

    # Get locations dictionary from json file
    path_this = os.path.realpath(os.path.dirname(__file__))

    try:
        file_path =  path_this + "/saved_locations.json"
        goal_dict = readJsonFile(file_path)
    except FileNotFoundError:
        exit("Error: saved_locations.json file not found")

    # Get objects dictionary from json file
    try:
        file_path = path_this + "/spawn_objects.json"
        object_dict = readJsonFile(file_path)
    except FileNotFoundError:
        exit("Error: spawn_objects.json file not found")

    # Get search objects from txt file
    try:
        with open(path_this+"/search_objects.txt",'r') as file:
            object_list = file.read().split("\n")
    except FileNotFoundError:
        exit("Error: search_objects.txt file not found")

    # Move to... coordinates/saved_location
    move_tab = menu_handler.insert( "Move to..." )
    menu_handler.insert("Go to coordinates...", parent=move_tab, callback=coordinatesCallback)

    for goal in goal_dict.keys():
        menu_handler.insert(goal, parent=move_tab, callback=partial(moveCallback, goal = goal, goal_dict = goal_dict))

    menu_handler.insert( "Stop", callback=partial(setSpeedCallback, linear=0, angular=0, text="Stopped"))

    # Spawn Object...
    spawn_tab = menu_handler.insert( "Spawn Object..." )

    for object_name in object_dict.keys():
        spawn_tab_2 = menu_handler.insert(object_name, parent=spawn_tab)
        menu_handler.insert("Random", parent=spawn_tab_2, callback=partial(spawnObjectCallback, model_name = object_name, object_dict = object_dict))
        for location in object_dict[object_name]:
            menu_handler.insert(location["name"], parent=spawn_tab_2, callback=partial(spawnObjectCallback, model_name = object_name, object_dict = object_dict, location = location["name"]))


    # Look for Object...
    search_tab = menu_handler.insert( "Look for Object..." )
    for object_name in object_list:

        search_tab_2 = menu_handler.insert(object_name, parent=search_tab)
        menu_handler.insert("Here", parent=search_tab_2, callback=partial(searchCallback, model_name = object_name, location = 0))
        search_tab_3 = menu_handler.insert("Room...", parent=search_tab_2)
    
        for goal in goal_dict.keys():
            menu_handler.insert(goal, parent=search_tab_3, callback=partial(searchCallback, model_name = object_name, location = goal, goal_dict = goal_dict))

        menu_handler.insert("Everywhere", parent=search_tab_2, callback=partial(searchCallback, model_name = object_name, location = 1, goal_dict = goal_dict))


    menu_handler.insert( "Take Picture", callback=takePictureCallback)

    # Other...
    other_tab = menu_handler.insert( "Other..." )

    menu_handler.insert( "Spin", parent=other_tab, callback=partial(setSpeedCallback, linear=0, angular=0.75, text="Spinning..."))
    menu_handler.insert( "Remove spawned objects", parent=other_tab, callback=partial(removeObjectsCallback, object_dict = object_dict))
    menu_handler.insert( "Remove Robutler", parent=other_tab, callback=removeRobotCallback)


def positionCallback(msg):
    # Always have a global variable with the current position
    global pose
    pose = msg.pose.pose


def imageCallback(msg):
    # Convert image to opencv

    global images
    try:
        images["camera"] = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return

    if images["camera"] is not None:
        cv2.imshow("Robutler's Camera", images["camera"])
        cv2.waitKey(1)

    # If an object is found, show it for 5 seconds
    if images["object"] is not None:
        cv2.imshow("Robutler's Camera", images["object"])
        images["object"] = None
        cv2.waitKey(5000)


def labelCallback(msg):
    # Save label list as global variable
    label_str = msg.data
    global labels
    labels = label_str.split("\n")


def yoloCallback(msg):
    # Save image in yolo topic as opencv image
    global images
  
    try:
        images["yolo"] = bridge.imgmsg_to_cv2(msg, "8UC3")
        #images["yolo"] = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return


def main():

    # Initialize
    rospy.init_node("menu")
    global server
    server = InteractiveMarkerServer("menu")
    menu_handler = MenuHandler()

    # Create menu
    initMenu(menu_handler)
    makeMenuMarker()
    menu_handler.apply(server, "menu_marker")
    makeTextMarker(text = "Ready")
    server.applyChanges()

    rospy.Subscriber('/odom', Odometry, positionCallback)
    rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
    rospy.Subscriber("/yolov7/yolov7_label", String, labelCallback)
    rospy.Subscriber("/yolov7/yolov7/visualization", Image, yoloCallback)


    # Spin until ctrl+c (can't use rospy.spin because of opencv)
    while not rospy.is_shutdown():
        rospy.sleep(10)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()