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
 *  @file      NodeParam.cpp
 *  @author    Sri Manika Makam
 *  @copyright 3-Clause BSD License
 *  @brief     Implementation of NodeParam class.
 */

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "NodeParam.hpp"

NodeParam::NodeParam() {
}

NodeParam::~NodeParam() {

}

std::vector<double> NodeParam::getCurrent() {
}

void NodeParam::setCurrent(std::vector<double> current) {
}

std::vector<double> NodeParam::getParent() {
}

void NodeParam::setParent(std::vector<double> parent) {
}

double NodeParam::getGCost() {
}

void NodeParam::setGCost(double costToGo) {
}

double NodeParam::getHCost() {
}

void NodeParam::setHCost(double costToCome) {
}

double NodeParam::getTotalCost() {
  return totalCost;
}

void NodeParam::setTotalCost(double total) {
}

double NodeParam::getTheta() {
}

void NodeParam::setTheta(double th) {
  theta = th;
}

double NodeParam::getDtheta() {
}

void NodeParam::setDtheta(double dth) {
}

double NodeParam::getDx() {
}

void NodeParam::setDx(double dX) {
}

double NodeParam::getDy() {
}

void NodeParam::setDy(double dY) {
}

