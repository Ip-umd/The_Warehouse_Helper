/**
 * The 3-Clause BSD License
 *
 * Copyright (c) 2019, Ishan Patel, Nakul Patel, Sri Manika Makam
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 */

/**
 *  @file      main.cpp
 *  @author    Ishan Patel
 *  @copyright 3-Clause BSD License
 *  @brief     main function for Acme's Warehouse Helper
 *
 *  This is the .cpp file that initializes the ROS node for publishing the velocities to topic,
 *  as well as define the start, intermediate and goal nodes in the warehouse map
 *  for the Turtlebot3 to navigate.
 */

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "AStar.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"



/**
 *  @brief main function for Warehouse Helper software
 *  @param argc-int count of arguments given through terminal
 *  @param argv-char** array of character pointers listing the arguments
 *  @return 0
 */
int main(int argc, char *argv[]) {
  /// Initializing the ROS node
  ros::init(argc, argv, "warehouseHelper");

  std::string startStation;
  std::string endStation;
  /// Prompting for the user input for start station
  std::cout << "Enter the start station : " << std::endl;
  std::cin >> startStation;
  /// Prompting for the user input for end station
  std::cout << "Enter the end station : " << std::endl;
  std::cin >> endStation;

  if (startStation == "A") {
    /// Defining the coordinates of station A
    std::vector<double> aStart { 520, 900 };
    std::vector<double> aEnd { 700, 450 };

    /// Defining the coordinates of station B
    std::vector<double> bStart { 700, 450 };
    std::vector<double> bEnd { 460, 630 };

    /// Defining the coordinates of station C
    std::vector<double> cStart { 460, 630 };
    std::vector<double> cEnd { 500, 200 };

    /// Creating an oject of AStar class
    AStar objAstar(10, 15, 1);

    /// Storing the backTrack function output for all stations
    std::vector<std::vector<std::vector<double>>> aPath = objAstar.aStar(aStart,
                                                                         aEnd);
    std::vector<std::vector<std::vector<double>>> bPath = objAstar.aStar(bStart,
                                                                         bEnd);
    std::vector<std::vector<std::vector<double>>> cPath = objAstar.aStar(cStart,
                                                                         cEnd);
    int i = 0;
    while (i < 10) {
      aPath.push_back({ { 0 }, { 0.168617 } });
      i = i + 1;
    }
    /// Appending rosInputs of B in A
    aPath.insert(std::end(aPath), std::begin(bPath), std::end(bPath));

    int j = 0;
    while (j < 10) {
      aPath.push_back({ { 0 }, { -0.4 } });
      j = j + 1;
    }
    /// Appending rosInputs of C in A
    aPath.insert(std::end(aPath), std::begin(cPath), std::end(cPath));

    int k = 0;
    while (k < 30) {
      aPath.push_back({ { 0 }, { 5.0 } });
      k = k + 1;
    }
    /// Calling the function for ROS publishing node
    objAstar.rosTurtle(aPath);
  }

  if (startStation == "B") {
    /// Defining the coordinates of station B
    std::vector<double> bStart { 700, 450 };
    std::vector<double> bEnd { 460, 630 };
    /// Defining the coordinates of station C
    std::vector<double> cStart { 460, 630 };
    std::vector<double> cEnd { 500, 200 };

    AStar objAstar(10, 15, 1);

    /// Storing the backTrack function output for B and C stations
    std::vector<std::vector<std::vector<double>>> bPath = objAstar.aStar(bStart,
                                                                         bEnd);
    std::vector<std::vector<std::vector<double>>> cPath = objAstar.aStar(cStart,
                                                                         cEnd);
    int j = 0;
    while (j < 10) {
      bPath.push_back({ { 0 }, { -0.4 } });
      j = j + 1;
    }
    /// Appending rosInputs of C in B
    bPath.insert(std::end(bPath), std::begin(cPath), std::end(cPath));

    int k = 0;
    while (k < 30) {
      bPath.push_back({ { 0 }, { 5.0 } });
      k = k + 1;
    }
    /// Calling the function for ROS publishing node
    objAstar.rosTurtle(bPath);
  }

  if (startStation == "C") {
    std::vector<double> cStart { 460, 630 };
    std::vector<double> cEnd { 500, 200 };

    AStar objAstar(10, 15, 1);

    /// Storing the backTrack function output for C station
    std::vector<std::vector<std::vector<double>>> cPath = objAstar.aStar(cStart,
                                                                         cEnd);
    int k = 0;
    while (k < 30) {
      cPath.push_back({ { 0 }, { 5.0 } });
      k = k + 1;
    }
    /// Calling the function for ROS publishing node
    objAstar.rosTurtle(cPath);
  }
  return 0;
}

