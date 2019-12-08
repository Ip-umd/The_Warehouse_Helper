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
 *  @file      AStar.cpp
 *  @author    Ishan Patel
 *  @copyright 3-Clause BSD License
 *  @brief     Implementation of AStar class
 */

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ObstacleMap.hpp"
#include "NodeParam.hpp"
#include "AStar.hpp"

AStar::AStar() {

}

AStar::AStar(double RPM1, double RPM2, double dT) {

}

AStar::~AStar() {
}

std::vector<std::vector<std::vector<double>>>  AStar::backTrack(
    std::vector<std::vector<std::vector<double>>>  backList) {
}

std::vector<std::vector<double>> AStar::motionModel() {

}

std::vector<std::vector<std::vector<double>>>  AStar::aStar(std::vector<double> startPoint,
                                              std::vector<double> goalPoint) {
}

