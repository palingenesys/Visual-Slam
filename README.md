# Visual-Slam

The idea is to integrate the stereo camera outputs (e.g. pointcloud and motion capture) into ros2 environment for enabeling VIO slam on both go2 and g1 unitree robots. 

## 1. Setup
For setting up the intelsense library follow the provided instruction by the official RealSense stereo camera repository:

```
cd ros2_ws
git clone https://github.com/realsenseai/librealsense
```
Depending on your operating system, you might follow different instructions for setting up all the required dependencies.
If you have ubuntu (reccomended to have set it up if not), follow the following link:

```
https://github.com/realsenseai/librealsense/blob/master/doc/installation.md
```

