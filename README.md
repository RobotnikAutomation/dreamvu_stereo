Publish PAL stereo images to ROS1 Topics

Code greatly abridge from DreamVu's `dreamvu_pal_navigation` package.

- First download PAL SDK and run install script: see  [DreamVu / PAL-USB](https://github.com/DreamVu/PAL-USB) 

- Git clone this package into catkin_ws/src

- Open a terminal and build the package:
```
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash
```
        
- To launch camera and tf nodes, and display stereo images in rviz, run
```
$ roslaunch dreamvu_stereo scan_rviz.launch
```
