# The-Warehouse-Helper
[ENPM - 808X Final Project ] Implementation of a robot that can pickup, transport and dropoff one or more loads between various known logistics stations based on user or system-generated orders that are not known a priori.

[![Build Status](https://travis-ci.org/Ip-umd/The_Warehouse_Helper.svg?branch=master)](https://travis-ci.org/Ip-umd/The_Warehouse_Helper)
[![Coverage Status](https://coveralls.io/repos/github/Ip-umd/The_Warehouse_Helper/badge.svg?branch=master)](https://coveralls.io/github/Ip-umd/The_Warehouse_Helper?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Overview

This repository is created as the part of ENPM808x Final Project to build a Warehouse Helper robot for ACME Robotics, which navigates through a known environment using path-planning module. The software implements autonomous navigation and mapping capability using ROS nodes and the simulated turtlebot platform.

Robots are becoming an integral component of our everyday life and being deployed in a number of processes over a wide range of applications. Such robots are required to find safe and feasible routes to navigate effectively in the environment.

The Turtlebot3, which we have used  as a base robot, uses a path-planning algorithm(A* algorithm) to navigate in a predefined map of warehouse. It then creates an optimal path from start station to goal station, while avoiding obstacles.

We also have implemented the funtionality that robot can navigate to intermediate stations between start station and goal location. We have considered four stations A,B,C and D. The user can given start point as A or B or C and end point as D. When given the start point as A and end point as D, the robot goes from A to B, B to C and then C to D. Accordingly, the same follows when given other start points. 

The completeion of task, will be shown by rotating/spinning the TurtleBot3 continuously at the goal station.


## Authors

Ishan Patel
 - Robotics student at University of Maryland
 - Areas of interests are perception and planning of self-driving vehicles.

Nakul Patel 
 - Robotics student at University of Maryland
 - Areas of interests are perception and planning of autonomous vehicles.
	
Sri Manika Makam
 - Robotics student at University of Maryland
 - Areas of interests are path-planning of autonomous vehicles.

## Google Sheet for AIP:

https://docs.google.com/spreadsheets/d/1t4cP0wDVoSsd_CHPAsWeQVUyKAmTfhGT0WIE1Eti5zc/edit?ts=5dd5b5d7#gid=1860513107

## Sprint Planning Notes:

https://docs.google.com/document/d/1rJrAD-c7H2RfS4iTaWQ11ydM7skeF1xUm2HMC1adbkw/edit?usp=sharing

## Presentation Slides and Demo Video:

The presentation slides are here in this link:
https://docs.google.com/presentation/d/1FcDADo-DJSLA1EpY_DcrcfGqXIMy3PhQI1eMom1GZ24/edit?usp=sharing

The DEMO video can be seen here: 
https://drive.google.com/file/d/16N9Nwl91wUtSwOp1dceJGjpwXtKHfuuX/view?usp=sharing



## Dependencies
For this project, you require following dependencies:

- Ubuntu 16.04
- ROS kinetic
- Gazebo 7.x
- Googletest
- OpenCV
- catkin
- Turtlebot packages

ROS can be installed from the https://wiki.ros.org site. Click on following link [here](https://wiki.ros.org/kinetic/Installation) to navigate to the installation guide for ROS.

To install Turtlebot packages, execute following command in terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers

```

- To Install Gazebo 7.x follow this link  [LINK](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0).

- To Install OpenCV follow this link [LINK](https://github.com/kyamagu/mexopencv/wiki/Installation-(Linux,-Octave,-OpenCV-3).

## License 

BSD 3-Clause License
```
Copyright (c) 2019, Ishan Patel, Nakul Patel, Sri Manika Makam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Package installation
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/Ip-umd/The_Warehouse_Helper.git
$ cd ..
$ catkin_make
```
## Instructions to run the program (DEMO)

To run the program first open a new terminal window and run following commands.
```
$ rosclean purge
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ roslaunch The_Warehouse_Helper warehouseHelperTest.launch x_pos:=-0.35 y_pos:=3.95
```
>[NOTE: **Make sure to run the source devel/setup.bash command evertime you want to run the package** otherwise the launch file won't detect the package. Also you will see **warnings** which our package will output, on the screen till gazebo is launched completely]

This command will ask for user input in following way:
```
 Enter the start station :
```
Here, type A 

then, following input will appear.
```
 Enter the goal station:
```
Here, type D.


If the start station is B, and goal station is D, then replace x_pos and y_pos value in roslaunch command by x_pos:=1.45 y_pos:=-0.55

If the start station is C, and goal station is D, then replace x_pos and y_pos value in roslaunch command by x_pos:=-0.95 y_pos:=1.25



## Instructions to run ROS Unit-Tests 
The tests are written using gtest and rostest. Close all the running processes before executing the commands below to run the rostest.

Following are the levels of tests are used in the package
- Level 1: unit tests
- Level 2: integration tests:


- main.cpp
- AStarTest.cpp
- NodeParamTestTest.cpp
- ObstacleMapTest.cpp
- warehouseHelperTest.launch

To run the tests, follow these commands:
In a new terminal,
```
roslaunch The_Warehouse_Helper warehouseHelperTest.launch
```
This command, will build a executable for tests named publishTest, which can be run using rosrun command.

In another terminal:
```
rosrun The_Warehouse_Helper warehouseHelper
```

## Doxygen Documentation

Although the repository contains the documentation, if you'd still like to generate it then follow the instructions below.

Install doxygen using below commands
```
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```
After installation run following command to open the doxywizard wherein you can fill in the details as required and set the source code folder to the repository as well. Create a new folder in the repository and select that as the destination directory. Add paths to include and src folders and then proceed with the default settings and generate the documentation.
```
doxywizard
```



