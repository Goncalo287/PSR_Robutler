#!/usr/bin/env python3

import move

import os
import rospy
import json

# Menus
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from functools import partial

# 2D Navigation Goal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Global variables
server = None
marker_pos = 0
menu_handler = MenuHandler()


# ----------------------------------
# Create box to detect right clicks
# ----------------------------------

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.1
    return marker


def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control


def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker


def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )



# ----------------------------------
#             Menus
# ----------------------------------


def readJsonFile(filename):
    with open(filename, 'r') as f:
        locations = json.load(f)
    return locations


def moveCallback( _ , goal):
    rospy.loginfo("Novo destino: " + goal)

    if isinstance(goal, str):
        try:
            file_name = os.path.realpath(os.path.dirname(__file__)) + '/saved_locations.json'
            goals_dict = readJsonFile(file_name)

            goal = goal.lower()
            goal_x = goals_dict[goal]["x"]
            goal_y = goals_dict[goal]["y"]
            goal_r = goals_dict[goal]["r"]

            move.movebaseClient(goal_x, goal_y, goal_r)
        except KeyError:
            print("Error: Unknown location")


def initMenu():

    # TODO: ir automaticamente ao ficheiro json buscar a lista de localizações guardadas para meter aqui
    # TODO: adicionar caixa de input para "Inserir coordenadas"
    # TODO: botão para cancelar a ordem atual

    move_tab = menu_handler.insert( "Mover" )
    menu_handler.insert( "Inserir coordenadas", parent=move_tab, callback=partial(moveCallback, goal = "????" ))
    menu_handler.insert( "Cozinha", parent=move_tab, callback=partial(moveCallback, goal = "Cozinha" ))
    menu_handler.insert( "Sala", parent=move_tab, callback=partial(moveCallback, goal = "Sala" ))
    menu_handler.insert( "Quarto", parent=move_tab, callback=partial(moveCallback, goal = "Quarto" ))


if __name__=="__main__":
    rospy.init_node("menu")

    server = InteractiveMarkerServer("menu")

    initMenu()
    
    makeMenuMarker( "marker" )
    menu_handler.apply( server, "marker" )

    server.applyChanges()
    rospy.spin()

