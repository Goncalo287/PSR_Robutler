<div align="center">

# ROS package for official YOLOv7

![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-blue?style=flat-square&logo=Ubuntu&logoColor=FFFFFF)
![Ros Noetic](https://img.shields.io/badge/Ros-Noetic-blue?style=flat-square&logo=ROS)
![Python 3.8](https://img.shields.io/badge/Python-3.8-blue?style=flat-square&logo=Python&logoColor=FFFFFF)

</div>

<font size=2>

> **Note** <br>
> This project if forked from [lukazso/yolov7-ros](https://github.com/lukazso/yolov7-ros)

</font>

<div align="center">
  <img src="docs/images/yolo_demo.gif" alt="Rviz" width="800"/>

  **Fig.1** RViz view of detection projection from a Gazebo Simulation
</div>

This repo contains a ROS noetic package for the official YOLOv7. It wraps the
[official implementation](https://github.com/WongKinYiu/yolov7) into a ROS node (so most credit
goes to the YOLOv7 creators).

<font size=2>

> **Note** <br>
> There are currently two YOLOv7 variants out there. This repo contains the
implementation from the paper [YOLOv7: Trainable bag-of-freebies sets new state-of-the-art for real-time object detectors](https://arxiv.org/abs/2207.02696).

</font>

## :rocket: Getting Started


Following ROS packages are required:
- [vision_msgs](http://wiki.ros.org/vision_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)

First, clone the repo into your catkin workspace and build the package:
```shell
git clone https://github.com/alexandrefch/yolov7-ros.git ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make_isolated
```

<font size=2>

> **Warning** <br>
> Be carefull when installing all requirements it's recommended to avoid install `opencv-python` from pip, compile it from source instead to make it work along with CUDA and opencv GStreamer addon. ([opencv GPU build script](https://github.com/mdegans/nano_build_opencv))

</font>

The Python requirements are listed in the `requirements.txt`. You can simply
install them as
```shell
pip install -r requirements.txt
```

Download the YOLOv7 weights from the [official repository](https://github.com/WongKinYiu/yolov7).

The package has been tested under Ubuntu 20.04 and Python 3.8.10.

## :clipboard: Usage
Before you launch the node, adjust the parameters in the
[launch file](launch/yolov7.launch). For example, you need to set the path to your
YOLOv7 weights and the image topic to which this node should listen to. The launch
file also contains a description for each parameter.

```shell
roslaunch yolov7_ros yolov7.launch
```

Each time a new image is received it is then fed into YOLOv7.

## :movie_camera: Visualization
You can visualize the yolo results if you set the `visualize` flag in the [launch file](launch/yolov7.launch). Also, with the `classes_path` parameter you can provide a `.txt` file with the class labels. An example file is provided in [berkeley.txt](class_labels/berkeley.txt) or [coco.txt](class_labels/coco.txt).

## :satellite: Topic
- The detections will be published under `/yolov7/out_topic` using the [vision_msgs/Detection2DArray](http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2DArray.html) message type.
- If you set the `visualize` parameter to `true`, the detections will be drawn into
  the image, which is then published under `/yolov7/out_topic/visualization`.

