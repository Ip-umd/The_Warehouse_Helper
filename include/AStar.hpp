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
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ObstacleMap.hpp"
#include "NodeParam.hpp"

/**
 * @brief AStar class
 * Class for the implementation of A* algorithm
 */
class AStar {
 private:
  NodeParam node;

  double rpm1;

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
   * @param dT of type double
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
   * @param backList - datatype to store the backtrack list of nodes
   * @return vector of vector of vector of double
   */
  std::vector<std::vector<std::vector<double>>> backTrack(
      std::vector<std::vector<std::vector<double>>> backList);

  /**
   * @brief motionModel - Function which describes the motion of robot
   * @param nodeInfo - none
   * @return vector of vector of double
   */
  std::vector<std::vector<double>> motionModel();

  /**
   * @brief motionModel - Function which implements the A* algorithm
   * @param startPoint - vector of double
   * @param goalPoint - vector of double
   * @return vector of vector of double
   */
  std::vector<std::vector<std::vector<double>>>  aStar(
        std::vector<double> startPoint, std::vector<double> goalPoint);
};


