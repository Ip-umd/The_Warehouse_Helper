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
 *  @file      ObstacleMap.hpp
 *  @author    Nakul Patel
 *  @copyright 3-Clause BSD License
 *  @brief     Header file for ObstacleMap.cpp
 *  Defines the member variables and functions in ObstacleMap class.
 */

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class ObstacleMap {
 private:
  double robotRadius;

 public:
  /**
   * lenght of map
   */
  int l;
  
  /**
   * width of map
   */
  int w;
  
  /**
   * @brief Default constructor of ObstacleMap class
   * @param none
   * @return none
   * Initializes private data members of the class to zero
   */
  ObstacleMap();

  /**
   * @brief Constructor of ObstacleMap class
   * @param x of type int
   * @param y of type int
   * @return none
   * Initializes x and y to the values passed to the constructor
   */
  ObstacleMap(int x, int y);

  /**
   * @brief Destructor of ObstacleMap class
   * @param none
   * @return none
   * Destroys class objects
   */
  ~ObstacleMap();

  /**
   * @brief setradius function
   * @param radius - double
   * @return none
   */
  void setRadius(double radius);

  /**
   * @brief getRadius function
   * @param none
   * @return double
   * Returns the robotRadius upon request
   */
  double getRadius();

  /**
   * @brief createRectangles - Function that creates obstacles of rectangle shape
   * @param x of type int
   * @param y of type int
   * @return bool
   */
  bool createRectangles(int x, int y);

  /**
   * @brief createTables - Function that creates tables as obstacles
   * @param x of type int
   * @param y of type int
   * @return bool
   */
  bool createTables(int x, int y);

  /**
   * @brief createCircles - Function that creates obstacles of circle shape
   * @param x of type int
   * @param y of type int
   * @return bool
   */
  bool createCircles(int x, int y);

  /**
   * @brief drawBoundary - Function that creates the boundaries of map
   * @param x of type int
   * @param y of type int
   * @return bool
   */
  bool drawBoundary(int x, int y);

  /**
   * @brief createMap- Function that creates the map
   * @param none
   * @return Mat present in cv
   */
  cv::Mat createMap();
};

