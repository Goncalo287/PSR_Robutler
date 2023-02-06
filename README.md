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
      * [Demo](#demo)
      * [Functionalities/Improvements](#functionalitiesimprovements)
  * [Authors](#authors)
  * [Reference](#reference)


# Description

Introducing the Robutler project, a project initiated in the [PSR (Robotic Systems Programming)](linkPSR) class at the [university of aveiro](https://www.ua.pt/) in the [Master's degree in mechanical engineering](https://www.ua.pt/pt/curso/488). <br/>
The goal of this project is to introduce students to the ROS (Robotic Operating System) communication system. By the end of the project, students acquired minimal but general competencies in using and communicating through the ROS system.

![image](https://user-images.githubusercontent.com/92520749/215944005-0af835c8-5634-4e37-bc28-ef263991ea8d.png)

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

## Demo

In this demonstration the group tries to demonstrate all the concepts represented below.

[video](https://github.com/jotadateta) 


## Functionalities/Improvements

- [x] Robot movimentation:
    - [x] using manual mode
    - [x] using diferent rooms
    - [x] using autonomous XY coordenates
    - [x] using autonomous information eg:"Kitchen"
    - [ ] using keyboard
- [x] Perception:
    - [x] find color spheres
    - [x] find objects
    - [x] find persons
    - [ ] meter outra cena
- [x] Objects:
    - [x] spawn objects random
    - [x] spawn specified object at random
    - [ ] orientation 

## Missions

- [x] Move to specified rooms
- [x] Robot search for spheres in selected room
- [x] Robot search for objects in selected room
- [x] Robot search for someone in selected room
- [x] Robot verify if table is cleaned or not
- [x] Robot photograph selected room
- [x] Robot search for someone in the appartement
- [x] Robot count the number of blue boxes in the appartment
- [x] escrever o que temos.
- [ ] escrever umas duas que nao tenhamos 
- [ ] escrever umas duas que nao tenhamos
    

 The color information will appear on the terminal where you run the program, as an approximation to the CSS21 list of colors as well as the actual RGB value. <br/>
 The dimensions will appear as a tuple such as (width, height) in meters.

Here we can see the extraction of images of objects used to train the classifier: <br/>
![image](https://user-images.githubusercontent.com/92520749/215945372-cfd947f6-9fe8-4e6c-9573-e4fdfc92bb5e.png)


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

