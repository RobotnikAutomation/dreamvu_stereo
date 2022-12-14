Publish PAL stereo images to ROS1 Topics

Code greatly abridge from DreamVu's `dreamvu_pal_navigation` package.

- First install CUDA and TensorRT using the instructions at  [DreamVu / PAL-USB](https://github.com/DreamVu/PAL-USB) 

- From [Dreamvu Software](https://dreamvu.com/software) download the `pal` file without GPU software:

PAL USB SDK CPU 	Intel i5/i7/i9 	Ubuntu 18.04/ 20.04 	4.1

- Run
```
cd ~/Downloads
chmod +x pal && ./pal
```

- Git clone this package into catkin_ws/src

- Open a terminal and build the package:
```
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash
```
        
- To launch camera and tf nodes, and display stereo images in rviz, run
```
$ roslaunch dreamvu_stereo stereo.launch
```
