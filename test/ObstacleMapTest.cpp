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
 *  @author    Nakul Patel
 *  @copyright 3-Clause BSD License
 *  @brief     unit tests for ObstacleMap class
 *
 *  This is the .cpp file that uses certain unit tests from
 *  gtest framework to test the functions of ObstacleMap class.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ObstacleMap.hpp"


/**
 * @brief Test for getRadius() function
 * @param ObstacleMapTest - name of the test suite 
 * @param GetRadiusTest - name of the test
 * @return none
 */
TEST( ObstacleMapTest, GetRadiusTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(10.0);
   EXPECT_EQ(obsMap.getRadius(), 10.0);
}


/**
 * @brief Test for createRectangles() function
 * @param ObstacleMapTest - name of the test suite 
 * @param CreateRectangleTest - name of the test
 * @return none
 */
TEST( ObstacleMapTest, CreateRectanglesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   /// Checking whether the coordinates are rectangles 
   bool req = obsMap.createRectangles(917,1069);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.createRectangles(1109,294);
   EXPECT_EQ(req1, true);
   bool req2 = obsMap.createRectangles(0,0);
   EXPECT_EQ(req2,false);
   bool req3 = obsMap.createRectangles(1111,296);
   EXPECT_EQ(typeid(req3), typeid(bool));
}


/**
 * @brief Test for createTables() function
 * @param ObstacleMapTest - name of the test suite 
 * @param CreateTablesTest - name of the test
 * @return none
 */
TEST( ObstacleMapTest, CreateTablesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   /// Checking whether the coordinates are tables
   bool req = obsMap.createTables(309,909);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.createTables(0,0);
   EXPECT_EQ(req1, false);
   bool req2 = obsMap.createTables(1111,296);
   EXPECT_EQ(typeid(req2), typeid(bool));
}

/**
 * @brief Test for createCircles() function
 * @param ObstacleMapTest - name of the test suite 
 * @param CreateCirclesTest - name of the test
 * @return none
 */
TEST( ObstacleMapTest, CreateCirclesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   /// Checking whether the coordinates are circles
   bool req = obsMap.createCircles(0,0);
   EXPECT_EQ(req, false);
   bool req1 = obsMap.createCircles(390, 965);
   EXPECT_EQ(req1,true);
   bool req2 = obsMap.createCircles(10,10);
   EXPECT_EQ(typeid(req2), typeid(bool));
}

/**
 * @brief Test for drawBoundary() function
 * @param ObstacleMapTest - name of the test suite
 * @param DrawBoundaryTest - name of the test  
 * @return none
 */
TEST( ObstacleMapTest, DrawBoundaryTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   /// Checking whether the coordinates are boundaries
   bool req = obsMap.drawBoundary(0,0);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.drawBoundary(555,250);
   EXPECT_EQ(req1,false);
   bool req2 = obsMap.drawBoundary(10,10);
   EXPECT_EQ(typeid(req2), typeid(bool));
}

/**
 * @brief Test for createMap() function
 * @param ObstacleMapTest - name of the test suite 
 * @param CreateMapTest - name of the test 
 * @return none
 * 
 */
TEST( ObstacleMapTest, CreateMapTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   /// Checking whether the coordinates are defined as obstacles
   cv::Mat req = obsMap.createMap();
   EXPECT_EQ(req.at<double>(0,1), 0);
   cv::Mat req1 = obsMap.createMap();
   EXPECT_EQ(req1.at<double>(20,20),1);
   cv::Mat req2 = obsMap.createMap();
   EXPECT_EQ(typeid(req2), typeid(cv::Mat));
}















