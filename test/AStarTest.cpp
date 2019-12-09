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
 *  @author    Ishan Patel
 *  @copyright 3-Clause BSD License
 *  @brief     unit tests for AStar class
 *
 *  This is the .cpp file that uses certain unit tests from
 *  gtest framework to test the functions of AStar class.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "AStar.hpp"

/**
 * @brief Test for backTrack() function
 * @param AStarTest - name of the test suite 
 * @param BackTrackTest - name of the test
 * @return none
 */
TEST( AStarTest, BackTrackTest) {
  AStar obj(25,40,1);
  std::vector<std::vector<std::vector<double>>> backList;
  backList.push_back({{1,2}, {0,0}, {1.0}, {0.1}});
  backList.push_back({{3,4},{1,2},{2.0},{0.2}});
  backList.push_back({{5,6},{3,4},{3.0},{0.3}});
  std::vector<std::vector<std::vector<double>>> rosInputs = obj.backTrack(backList);
  EXPECT_EQ(rosInputs.at(0).at(0).at(0), 1.0);
  EXPECT_EQ(rosInputs.at(0).at(1).at(0), 0.1);   
}  


/**
 * @brief Test for motionModel() function
 * @param AStarTest - name of the test suite 
 * @param MotionModelTest - name of the test
 * @return none
 */
TEST( AStarTest, MotionModelTest) {
  AStar obj(25,40,1);
  std::vector<std::vector<double>> actions = obj.motionModel();
  EXPECT_NEAR(actions.at(0).at(0), 5, 0.001);
  EXPECT_NEAR(actions.at(0).at(1), -1, 0.001);
  EXPECT_NEAR(actions.at(0).at(2), 5.09902, 0.001);
  EXPECT_NEAR(actions.at(0).at(3), -0.281028, 0.001);
  EXPECT_NEAR(actions.at(0).at(4), 4.97419, 0.001);
  EXPECT_NEAR(actions.at(0).at(5), -0.281028, 0.001);
}


/**
 * @brief Test for aStar() function
 * @param AStarTest - name of the test suite 
 * @param AStarFuncTest - name of the test
 * @return none
 */
TEST( AStarTest, AStarFuncTest) {
  AStar obj(25,40,1);
  std::vector<double> stp{3,3};
  std::vector<double> etp{7,7};
  std::vector<std::vector<std::vector<double>>> dif = obj.aStar(stp,etp);
  EXPECT_NEAR(dif.at(0).at(0).at(0), 0, 0.0001);
  EXPECT_NEAR(dif.at(0).at(1).at(0), 0, 0.0001);
  EXPECT_NEAR(dif.at(1).at(0).at(0), 4.97419, 0.0001);
  EXPECT_NEAR(dif.at(1).at(1).at(0), 0.281028, 0.0001);
}

/**
 * @brief callBack function for subscriber 
 * @param vel - velocity of type geometry_msgs::Twist 
 * @return none
 */
void velCallback(geometry_msgs::Twist vel) {
}

/**
 * @brief Test for rosTurtle() function
 * @param AStarTest - name of the test suite 
 * @param RosTurtleTest - name of the test
 * @return none
 */
TEST( AStarTest, RosTurtleTest) {
  ros::NodeHandle n;
  ros::Publisher testPub = n.advertise < geometry_msgs::Twist > ("/cmd_vel", 1000);    
  ros::Subscriber testSub = n.subscribe("cmd_vel", 1000, velCallback);
  EXPECT_EQ(testPub.getNumSubscribers(), 1);
  EXPECT_EQ(testSub.getNumPublishers(), 1);
 }












