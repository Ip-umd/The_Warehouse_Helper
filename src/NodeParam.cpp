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
#include "NodeParam.hpp"

NodeParam::NodeParam() {
  gCost = 0.0;
  hCost = 0.0;
  totalCost = 0.0;
  theta = 0.0;
  dtheta = 0.0;
  dx = 0.0;
  dy = 0.0;
}

NodeParam::~NodeParam() {
}

std::vector<double> NodeParam::getCurrent() {
  /// returns currentNode
  return currentNode;
}

void NodeParam::setCurrent(std::vector<double> current) {
  /// setting the value of currentNode
  currentNode = current;
}

std::vector<double> NodeParam::getParent() {
  /// returns parentNode
  return parentNode;
}

void NodeParam::setParent(std::vector<double> parent) {
  /// setting the value of parentNode
  parentNode = parent;
}

double NodeParam::getGCost() {
  /// returns gCost
  return gCost;
}

void NodeParam::setGCost(double costToGo) {
  /// sets the value of gCost
  gCost = costToGo;
}

double NodeParam::getHCost() {
  /// returrns hCost
  return hCost;
}

void NodeParam::setHCost(double costToCome) {
  /// sets the value of hCost
  hCost = costToCome;
}

double NodeParam::getTotalCost() {
  /// returns totalCost
  return totalCost;
}

void NodeParam::setTotalCost(double total) {
  /// sets the value of totalCost
  totalCost = total;
}

double NodeParam::getTheta() {
  /// returns theta
  return theta;
}

void NodeParam::setTheta(double th) {
  /// sets the value of theta
  theta = th;
}

double NodeParam::getDTheta() {
  /// returns dtheta
  return dtheta;
}

void NodeParam::setDTheta(double dth) {
  /// sets the value of dtheta
  dtheta = dth;
}

double NodeParam::getDx() {
  /// returns dx
  return dx;
}

void NodeParam::setDx(double dX) {
  /// sets the value of dx
  dx = dX;
}

double NodeParam::getDy() {
  /// returns dy
  return dy;
}

void NodeParam::setDy(double dY) {
  /// sets the value of dy
  dy = dY;
}



