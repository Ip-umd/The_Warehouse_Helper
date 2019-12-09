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
 *  @file      AStar.hpp
 *  @author    Ishan Patel
 *  @copyright 3-Clause BSD License
 *  @brief     Header file for AStar.cpp
 *  Defines the member variables and functions in AStar class.
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "NodeParam.hpp"
#include "ObstacleMap.hpp"


/**
 * @brief AStar class
 * Class for the implementation of A* algorithm
 */
class AStar {
 private: 
  /// establishing part-of relationship by using object of NodeParam class
  NodeParam node;
  /// wheel velocity of robot
  double rpm1;
  /// wheel velocity of robot
  double rpm2;

  double dt;

 public:

  /**
   * @brief Default constructor of AStar class
   * @param none
   * @return none
   * Initializes rpm1,rpm2 and dt to zero
   */
  AStar();

  /**
   * @brief Constructor of AStar class
   * @param rpm1Val of type double
   * @param rpm2Val of type double
   * @param dtVal of type double
   * @return none
   * Initializes rpm1, rpm2 and dt to the values passed to the constructor
   */
  AStar(double rpm1Val, double rpm2Val, double dtVal);
  

  /**
   * @brief Destructor of AStar class
   * @param none
   * @return none
   * Destroys class objects
   */
  ~AStar();


  /**
   * @brief backtrack - Function which implements the backtracking
   * @param backList - contains currentNode, parentNode, velocity 
   *                   and angle for nodes
   * @return velocity and angles for nodes in the obtained path
   */
  std::vector<std::vector<std::vector<double>>> backTrack(
		  std::vector<std::vector<std::vector<double>>> backList);


  /**
   * @brief motionModel - Function which describes the motion of robot
   * @param none
   * @return std::vector<std::vector<double>> - action states for a node
   */
  std::vector<std::vector<double>> motionModel();


  /**
   * @brief motionModel - Function which implements the A* algorithm
   * @param startPoint - coordinates of startPoint
   * @param goalPoint - coordinates of goalPoint
   * @return velocities and angles of nodes in backtrack list
   */
  std::vector<std::vector<std::vector<double>>> aStar(
      std::vector<double> startPoint, std::vector<double> goalPoint);


  /**
   * @brief motionModel - Function for publlishing to ROS topic
   * @param rosInputs - variable having velocities and angles from backTrack function
   * @return none
   *
   * This function creates a ROS node, which publishes the velocities and angles to 
   * the cmd_vel topic, which Turtlebot uses to navigate on the path 
   * returned by A* algorithm
   */
  void rosTurtle(std::vector<std::vector<std::vector<double>>> rosInputs);
};






