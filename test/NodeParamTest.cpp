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
 *  @file      ObstacleMapTest.cpp
 *  @author    Sri Manika Makam
 *  @copyright 3-Clause BSD License
 *  @brief     unit tests for ObstacleMap class
 *
 *  This is the .cpp file that uses certain unit tests from
 *  gtest framework to test the functions of NodeParam class.
 */


#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include "NodeParam.hpp"


/**
 * @brief Test for all getter funtions for member variables
 * @param NodeParamTest - name of the test suite
 * @param GettersTest - name of the test
 * @return none
 */
TEST(NodeParamTest, GettersTest) {
  NodeParam node;
  node.setCurrent({ 10, 20 });
  std::vector<double> curr = { 10, 20 };
  EXPECT_EQ(node.getCurrent(), curr);
  node.setParent({ 20, 25 });
  std::vector<double> par = { 20, 25 };
  EXPECT_EQ(node.getParent(), par);
  node.setGCost(25.0);
  EXPECT_EQ(node.getGCost(), 25.0);
  node.setHCost(10.0);
  EXPECT_EQ(node.getHCost(), 10.0);
  node.setTotalCost(35.0);
  EXPECT_EQ(node.getTotalCost(), 35.0);
  node.setTheta(5.0);
  EXPECT_EQ(node.getTheta(), 5.0);
  node.setDTheta(2.0);
  EXPECT_EQ(node.getDTheta(), 2.0);
  node.setDx(0.5);
  EXPECT_EQ(node.getDx(), 0.5);
  node.setDy(0.5);
  EXPECT_EQ(node.getDy(), 0.5);
}






