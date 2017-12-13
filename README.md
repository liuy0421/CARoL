## Final Project: CARoL, Coordinated, Automated Robot of Levity
COMP150-02 Developmental Robotics;
Fall 2017;
Team Members: Azmina Karukappadath (https://github.com/akaruk01), Sam Weiss (https://github.com/SamuelWeiss), and Yuelin Liu (https://github.com/liuy0421)

## Installation
CARoL requires the installation of:
1) Ubuntu 14.04, and
2) ROS Indigo Igloo (http://wiki.ros.org/indigo)

## Setup
First, create a workspace and clone the source repository.

```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/liuy0421/CARoL.git
```

Next, install all dependencies:
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Then, build:
```
$ catkin_make
$ source devel/setup.bash
```

## Launch a TurtleBot2

After everything is installed and made sucessfully, in the first terminal, do:

```
$ roslaunch tbot2_launch tbot2.launch
```

This will launch the robot's drivers. At this step, check the terminal output to ensure that the 3D vision sensor is found correctly (there is a bug in the openni driver where sometimes it doesn't). If a message about not finding an opeeni device keeps repearting, then ctrl-c the processes, and unplug the 3D sensor and plug it in again, and try again. 

In a second terminal, do:


```
$ roslaunch tbot2_launch amcl_navigation.launch
```

After rviz shows up, provide the robot's initial location.

Then, in a third terminal, do:

```
$ rosrun tbot2_roam tbot2.roam
```

And your favorite Turtlebot should be ready to bring joy to the world!


Special thanks to Jivko Sinapov (https://github.com/jsinapov) for providing guidance over the course of this project.
