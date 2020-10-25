# CppND-Capstone-Turtlebot3

**Prerequisites:**
* Ubuntu 20.04

## Install ROS Noetic
http://wiki.ros.org/noetic/Installation/Ubuntu

## Create Catkin Workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws
$ catkin_make

```
## Set environment variables and source workspace
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
$ source ~/.bashrc

```
If you want to run the Gazebo simulation on a VMware virtual machine, the following variable needs to be set as well:
```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
```

## Clone this Repo
```
$ cd src
$ git clone https://github.com/verena-roehrl/CppND-Capstone-Turtlebot3.git
$ cd CppND-Capstone-Turtlebot3
```
TODO submodules update




für jeden turtlebot eigene Klasse
Wahl zwischen teleoperation vs navigation

create catkin_ws

https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/

nur Manager starten
Manager startet neue nodes -> spawnt turtlebots
