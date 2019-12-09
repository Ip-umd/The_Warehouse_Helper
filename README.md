# The-Warehouse-Helper
[ENPM - 808X Final Project ] Implementation of a robot that can pickup, transport and dropoff one or more loads between various known logistics stations based on user or system-generated orders that are not known a priori.

[![Build Status](https://travis-ci.org/Ip-umd/The_Warehouse_Helper.svg?branch=iteration1)](https://travis-ci.org/Ip-umd/The_Warehouse_Helper)
[![Coverage Status](https://coveralls.io/repos/github/Ip-umd/The_Warehouse_Helper/badge.svg?branch=iteration1)](https://coveralls.io/github/Ip-umd/The_Warehouse_Helper?branch=iteration1)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Overview

We have developed a Warehouse Helper robot for Acme Robotics. The Turtlebot3, which we have used  as a base robot, uses a path-planning algorithm(A* algorithm) to navigate in a predefined map of warehouse. It then creates an optimal path from start station to goal station, while avoiding obstacles.

We also have implemented the funtionality that robot can navigate to intermediate stations between start station and goal location.


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
$ roslaunch The_Warehouse_Helper warehouseHelperTest.launch
```
>[NOTE: **Make sure to run the source devel/setup.bash command evertime you want to run the package** otherwise the launch file won't detect the package. Also you will see **warnings** which our package will output, on the screen till gazebo is launched completely]


## Instructions to run ROS Unit-Tests 
The tests are written using gtest and rostest. Close all the running processes before executing the commands below to run the rostest.

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

## Working with Eclipse IDE

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)

```
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/Ip-umd/The_Warehouse_Helper.git
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of The_Warehouse_Helper

```
cd ~/workspace
mkdir -p The_Warehouse_Helper_Eclipse
cd The_Warehouse_Helper_Eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../The_Warehouse_Helper/
```


