# PSR_Robutler

Robutler project introduces the world of robotic system simulation. It works in tandem with the Gazebo environment simulator to simulate an apartment. The Robutler project facilitates communication between the entire system and ROS, a collection of frameworks that simplify the process of simulating robotic systems.

<p align="center">
  <img src="https://www.hipersuper.pt/wp-content/uploads/2012/12/Universidade-de-Aveiro.jpg">
</p>

Index
=================

  * [Description](#description)
  * [The Project](#the-project)
      * [Requirements](#requirements)
      * [Ambient/apartment](#Ambient/apartment)
      * [Usage](#usage)
      * [Explained](#Explained)
      * [Functionalities/Improvements](#functionalitiesimprovements)
  * [Authors](#authors)
  * [Reference](#reference)


# Description

Introducing the Robutler project, a project initiated in the [PSR (Robotic Systems Programming)](linkPSR) class at the [university of aveiro](https://www.ua.pt/) in the [Master's degree in mechanical engineering](https://www.ua.pt/pt/curso/488). <br/>
The goal of this project is to introduce students to the ROS (Robotic Operating System) communication system. By the end of the project, students acquired minimal but general competencies in using and communicating through the ROS system.

![image](https://media.discordapp.net/attachments/1062801396921016373/1072259141466345502/image.png?width=1263&height=660)

# The Project
The project utilizes Gazebo for simulating the environment, RViz for the graphical interface, YOLO for object identification, and ROS for communication. These tools provide a comprehensive simulation platform that allows for accurate representation of the environment and efficient communication between different components. YOLO is used to identify objects within the simulation, while ROS acts as the underlying communication layer to facilitate communication between the various modules. Overall, the combination of these tools provides a powerful and flexible solution for simulating and analyzing complex systems.

## Requirements
It is necessary to install the following resources before any use:
* [ROS](http://wiki.ros.org/ROS/Installation)
* [OpenCV](https://pypi.org/project/opencv-python/) 
* Gazebo (included with ROS)
* [YOLO](https://github.com/alexandrefch/yolov7-ros)
* RVIZ (included with ROS)

## Ambient/apartment
The apartment map is incluied with the project, above you can see the plant of the apartment for better compreention.
[apartment repository](https://github.com/miguelriemoliveira/psr_22-23/tree/master/TrabalhoRobutler).

<p align="center">
  <img src="https://github.com/miguelriemoliveira/psr_22-23/blob/master/TrabalhoRobutler/docs/floorplan.jpg?raw=true">
</p>

* For the furniture you need to install two package:
```
    cd catkin_ws/src
    git clone https://github.com/aws-robotics/aws-robomaker-small-house-world
    git clone https://github.com/aws-robotics/aws-robomaker-hospital-world
```
*Dont forget to compile with catkin_make after this installation.*


## Usage

First you need to launch gazebo:
```
roslaunch robutler_bringup gazebo.launch
```

next you spawn robot, it cames incluied with all the launch files:
```
roslaunch robutler_bringup bringup.launch
```

### Edit saved map
Start editor:
```
rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint
```
Save new map:
```
roscd robutler_navigation/maps/map/ && rosrun map_server map_saver -f map
```

## Explained
### Interface
* In the center of the image, there is a robot and a mapped apartment with a box, indicating its operational state. On the upper right, there is an image captured by the robot. On the lower right, there is an image processed by YOLO (You Only Look Once).

![image](https://media.discordapp.net/attachments/1062801396921016373/1072259404155600977/Screenshot_from_2023-02-06_20-55-49.png?width=1215&height=660)

* In this image, we can understand the RVIZ described above. On the right is an image using OpenCV. On the bottom right is the manual control of the robot. On the bottom left is Gazebo, which simulates the real environment.

![image](https://media.discordapp.net/attachments/1062801396921016373/1072260532503052429/Screenshot_from_2023-02-06_21-00-15.png?width=1260&height=660)

### Demo
In this demonstration the group tries to demonstrate all the concepts represented below.

[video](https://github.com/jotadateta) 


## Functionalities/Improvements

- [x] Robot movimentation:
    - [x] using manual mode
    - [x] using diferent rooms
    - [x] using autonomous XY coordenates(input box)
    - [x] using autonomous information eg:"Kitchen"
    - [x] using autonomous searching
    - [ ] using keyboard
- [x] Perception:
    - [x] find color spheres
    - [x] find objects
    - [x] find persons
    - [ ] Count different objets(with same properties)
- [x] Objects:
    - [x] spawn objects random
    - [x] spawn specified object at random
    - [x] spawn specified object at list of spots
    - [ ] spawn objects at click 

## Missions

- [x] Move to specified rooms
- [x] Robot search for spheres in selected room/everywhere
- [x] Robot search for objects in selected room/everywhere
- [x] Robot search for someone in selected room/everywhere
- [ ] Robot verify if table is cleaned or not
- [x] Robot photograph selected room
- [x] Robot search for someone in the appartement
- [ ] Robot count the number of blue boxes in the appartment(50%)
- [ ] Robot touch objects.
- [ ] Robot move objects   
    


# Authors

- [@jotadateta](https://github.com/jotadateta) - joaopedrotomas@ua.pt 93366
- [@Goncalo287](https://github.com/Goncalo287) - goncalo.anacleto@ua.pt 93394
- [@cmrsantos](https://github.com/cmrsantos) - cmrsantos@ua.pt 92955
- [@joaof_2025](https://github.com/JoaoF2025) - 
joaolmf@ua.pt 93296

 
# Reference

 - [Proposed work](https://github.com/miguelriemoliveira/psr_22-23/tree/master/TrabalhoRobutler)

- [YOLOv7-ROS](https://github.com/alexandrefch/yolov7-ros)

- [YOLOv7-trained](https://github.com/WongKinYiu/yolov7)

