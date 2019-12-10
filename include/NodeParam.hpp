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
 *  @file      NodeParam.hpp
 *  @author    Sri Manika Makam
 *  @copyright 3-Clause BSD License
 *  @brief     Header file for NodeParam.cpp
 *  Defines the member variables and functions in NodeParam class.
 */

#ifndef INCLUDE_NODE_PARAM_HPP_
#define INCLUDE_NODE_PARAM_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>


/**
 * @brief NodeParam class
 * Class for the implementation of costs, velocities, angles
 * and coordinates for nodes in A* algorithm
 */
class NodeParam {
 private:
  std::vector<double> currentNode;
  std::vector<double> parentNode;
  double gCost;
  double hCost;
  double totalCost;
  double theta;
  double dtheta;
  double dx;
  double dy;

 public:
  /**
   * @brief Default constructor of NodeParamclass
   * @param none
   * @return none
   * Initializes all the private data members of the class to zero
   */
  NodeParam();


  /**
   * @brief Destructor of NodeParam class
   * @param none
   * @return none
   * Destroys class objects
   */
  ~NodeParam();

  /**
   * @brief getCurrent function
   * @param none
   * @return std::vector<double> - currentNode
   * Returns the currentNode upon request
   */
  std::vector<double> getCurrent();

  /**
   * @brief setCurrent function
   * @param std::vector<double> - current
   * @return none
   */
  void setCurrent(std::vector<double> current);

  /**
   * @brief getParent function
   * @param none
   * @return std::vector<double> - parentNode
   * Returns the parentNode upon request
   */
  std::vector<double> getParent();

  /**
   * @brief setParent function
   * @param std::vector<double> - parent
   * @return none
   */
  void setParent(std::vector<double> parent);

  /**
   * @brief getGCost function
   * @param none
   * @return double - gCost
   * Returns the gCost upon request
   */
  double getGCost();

  /**
   * @brief setGCost function
   * @param double - costToGo
   * @return none
   */
  void setGCost(double costToGo);

  /**
   * @brief getHCost function
   * @param none
   * @return double - hCost
   * Returns the hCost upon request
   */
  double getHCost();

  /**
   * @brief setHCost function
   * @param double - costToCome 
   * @return none
   */
  void setHCost(double costToCome);

  /**
   * @brief getTotalCost function
   * @param none
   * @return double - totalCost
   * Returns the totalCost upon request
   */
  double getTotalCost();

  /**
   * @brief setTotalCost function
   * @param double - total
   * @return none
   */
  void setTotalCost(double total);

  /**
   * @brief getTheta function
   * @param none
   * @return double - theta
   * Returns the theta upon request
   */
  double getTheta();

  /**
   * @brief setTheta function
   * @param double - th
   * @return none
   */
  void setTheta(double th);

  /**
   * @brief getDTheta function
   * @param none
   * @return double -dtheta
   * Returns the dtheta upon request
   */
  double getDTheta();

  /**
   * @brief setDTheta function
   * @param double - dth 
   * @return none
   */
  void setDTheta(double dth);

  /**
   * @brief getDx function
   * @param none
   * @return double - dx
   * Returns the dx upon request
   */
  double getDx();

  /**
   * @brief setDx function
   * @param double - dX
   * @return none
   */
  void setDx(double dX);

  /**
   * @brief getDy function
   * @param none
   * @return double - dy
   * Returns the dy upon request
   */
  double getDy();

  /**
   * @brief setDy function
   * @param double - dY
   * @return none
   */
  void setDy(double dY);
};

#endif  // INCLUDE_NODE_PARAM_HPP_
