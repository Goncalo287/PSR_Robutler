# PSR_Robutler


### Start

Launch gazebo:
```
roslaunch robutler_bringup gazebo.launch
```

Spawn robot:
```
roslaunch robutler_bringup bringup.launch
```




### Edit saved map
Start:
```
rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint
```
Save:
```
roscd robutler_navigation/maps/map/ && rosrun map_server map_saver -f map
```
