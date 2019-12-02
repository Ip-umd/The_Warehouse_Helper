# The-Warehouse-Helper
[ENPM - 808X Final Project ] Implementation of a robot that can pickup, transport and dropoff one or more loads between various known logistics stations based on user or system-generated orders that are not known a priori.

[![Build Status](https://travis-ci.org/Ip-umd/The_Warehouse_Helper.svg?branch=iteration1)](https://travis-ci.org/Ip-umd/The_Warehouse_Helper)
[![Coverage Status](https://coveralls.io/repos/github/Ip-umd/The_Warehouse_Helper/badge.svg?branch=iteration1)](https://coveralls.io/github/Ip-umd/The_Warehouse_Helper?branch=iteration1)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

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

## Standard install via command-line
```
git clone https://github.com/Ip-umd/The_Warehouse_Helper.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
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

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> Select "The_Warehouse_Helper_Eclipse" directory created previously as root directory -> Finish

## Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.

## Build

To build the project, in Eclipse, unfold The_Warehouse_Helper_Eclipse project in Project Explorer, unfold Build Targets, double click on "all" to build all projects.

## Run

In Eclipse, right click on the The_Warehouse_Helper_Eclipse in Project Explorer, select Run As -> Local C/C++ Application

Choose the binaries to run (e.g. shell-app, cpp-test for unit testing)

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
