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

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

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
   * @return vector of double
   * Returns the currentNode upon request
   */
  std::vector<double> getCurrent();

  /**
   * @brief setCurrent function
   * @param current - vector of double
   * @return none
   */
  void setCurrent(std::vector<double> current);

  /**
   * @brief getparent function
   * @param none
   * @return vector of double
   * Returns the parentNode upon request
   */
  std::vector<double> getParent();

  /**
   * @brief setParent function
   * @param parent - vector of double
   * @return none
   */
  void setParent(std::vector<double> parent);

  /**
   * @brief getGCost function
   * @param none
   * @return double
   * Returns the gCost upon request
   */
  double getGCost();

  /**
   * @brief setGCost function
   * @param costToGo - double
   * @return none
   */
  void setGCost(double costToGo);

  /**
   * @brief getHCost function
   * @param none
   * @return double
   * Returns the hCost upon request
   */
  double getHCost();

  /**
   * @brief setHCost function
   * @param costToCome - double
   * @return none
   */
  void setHCost(double costToCome);

  /**
   * @brief getTotalCost function
   * @param none
   * @return double
   * Returns the totalCost upon request
   */
  double getTotalCost();

  /**
   * @brief settotalCost function
   * @param total - double
   * @return none
   */
  void setTotalCost(double total);

  /**
   * @brief getTheta function
   * @param none
   * @return double
   * Returns the theta upon request
   */
  double getTheta();

  /**
   * @brief setTheta function
   * @param th - double
   * @return none
   */
  void setTheta(double th);

  /**
   * @brief getDTheta function
   * @param none
   * @return double
   * Returns the dtheta upon request
   */
  double getDTheta();

  /**
   * @brief setDTheta function
   * @param dth - double
   * @return none
   */
  void setDTheta(double dth);

  /**
   * @brief getDx function
   * @param none
   * @return double
   * Returns the dx upon request
   */
  double getDx();

  /**
   * @brief setDx function
   * @param dX - double
   * @return none
   */
  void setDx(double dX);

  /**
   * @brief getDy function
   * @param none
   * @return double
   * Returns the dy upon request
   */
  double getDy();

  /**
   * @brief setDy function
   * @param dY - double
   * @return none
   */
  void setDy(double dY);
};

