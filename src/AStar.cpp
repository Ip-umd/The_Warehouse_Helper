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

#include "AStar.hpp"
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


/**
 * declaring typedef for std::vector<std::vector<std::vector<double>>>
 */
typedef std::vector<std::vector<std::vector<double>>> nestVector;

AStar::AStar() {
}

AStar::AStar(double rpm1Val, double rpm2Val, double dtVal) {
  /// Setting the values of member variables
  rpm1 = rpm1Val;
  rpm2 = rpm2Val;
  dt = dtVal;
}

AStar::~AStar() {
}


nestVector AStar::backTrack(
    std::vector<std::vector<std::vector<double>>> backList) {
  std::vector<std::vector<double>> path;
  nestVector rosInputs;
  int length = backList.size();
  /// storing the currentPos of node
  std::vector<double> currentPos = backList[length - 1][0];
  /// appending velocity and angle
  rosInputs.push_back({ backList[length - 1][2], backList[length - 1][3] });
  path.push_back(currentPos);
  /// getting the parent from backList
  std::vector<double> parent = backList[length - 1][1];
  std::vector<double> zero;
  zero.push_back(0);
  zero.push_back(0);
  while (parent != zero) {
    for (int i = 0; i < length; i++) {
      std::vector < std::vector<double> > X = backList[i];
      /// checking if parent found
      if (X[0] == parent) {
        parent = X[1];
        currentPos = X[0];
        path.push_back(currentPos);
        /// appending velocity and angle in rosInputs
        rosInputs.push_back({ { X[2] }, { X[3] } });
      }
    }
  }
  /// reversing the data stored in path and rosInputs
  std::reverse(path.begin(), path.end());
  std::reverse(rosInputs.begin(), rosInputs.end());
  return rosInputs;
}



std::vector<std::vector<double>> AStar::motionModel() {
  /// defining the distance between wheels of Turtlebot3
  double l = 35.4;
  /// defining the radius of wheels of Turtlebot3
  double r = 3.8;

  std::vector<std::vector<double>> action;
  /// 8 possible action states for a node based on rpm's
  std::vector<std::array<double, 2>> steps { { 0.0, rpm1 }, { rpm1, 0.0 }, {
      rpm1, rpm1 }, { 0.0, rpm2 }, { rpm2, 0.0 }, { rpm2, rpm2 },
      { rpm1, rpm2 }, { rpm2, rpm1 } };

  double theta = node.getTheta();

  for (auto motion : steps) {
    /// defining ur, ul and  thetadot based on differential drive model
    double ur = (motion.at(0) * M_PI) / 30;
    double ul = (motion.at(1) * M_PI) / 30;
    double thetadot = (r / l) * (ur - ul);
    /// calculating change in angle
    double dtheta = thetadot * dt;
    double change_theta = theta + dtheta;

    double xdot = (r / 2) * (ul + ur) * cos(change_theta);
    double ydot = (r / 2) * (ul + ur) * sin(change_theta);
    /// setting the dx for node
    double dx = round(xdot * dt);
    /// setting the dy for node
    double dy = round(ydot * dt);
    /// setting the cost
    double cost = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));
    /// setting velocity magnitude
    double vel_mag = std::sqrt(std::pow(xdot, 2) + std::pow(ydot, 2));

    action.push_back({ dx, dy, cost, change_theta, vel_mag, dtheta });
  }
  return action;
}

nestVector AStar::aStar(std::vector<double> startPoint,
                        std::vector<double> goalPoint) {
  /// defining struct to implement the priority queue
  struct Compare {
    bool operator()(std::vector<std::vector<std::vector<double>>> const & a,
                    std::vector<std::vector<std::vector<double>>> const & b) {
      return a[0][0][0] > b[0][0][0];
    }
  };
  /// declaring the variables for storing unvisited and visited nodes
  std::priority_queue<std::vector<std::vector<std::vector<double>>>,
      std::vector<std::vector<std::vector<std::vector<double> >>>,
        Compare> unvisited;
  std::priority_queue<std::vector<std::vector<std::vector<double>>>,
      std::vector<std::vector<std::vector<std::vector<double> >>>,
          Compare> visited;

  double theta = 0.0;
  /// defining the startNode
  nestVector startNode;
  startNode.push_back({ { 0.0 }, startPoint, { 0, 0 }, { 0.0 }, { theta }, {
      0.0 }, { 0.0 } });
  /// defining the endNode
  nestVector endNode;
  endNode.push_back({ { 0.0 }, goalPoint, { 0, 0 }, { 0.0 }, { theta },
      { 0.0 }, { 0.0 } });

  unvisited.push({ { { 0.0 }, startPoint, { 0, 0 }, { 0.0 }, { theta },
      { 0.0 }, { 0.0 } } });

  ObstacleMap map(1110, 1010);
  /// setting the radius of robot
  map.setRadius(50.0);
  /// Creating costMap for node exploration
  cv::Mat costMap = cv::Mat::ones(1110, 1010, CV_64F);
  costMap.at<double>(startNode.at(0).at(1).at(0),
                     startNode.at(0).at(1).at(1)) = 0;
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::resizeWindow("Display frame", 505, 555);
  cv::imshow("Display window", costMap);
  cv::waitKey(1);

  nestVector path;
  nestVector bT;

  while (unvisited.size() > 0) {
    nestVector currentNode;
    /// storing the node with least cost in currentNode
    currentNode = unvisited.top();
    /// removing the node from unvisited
    unvisited.pop();
    /// adding the node in visited
    visited.push(currentNode);

    bT.push_back({ currentNode.at(0).at(1), currentNode.at(0).at(2),
        currentNode.at(0).at(5), currentNode.at(0).at(6) });

    /// defining the threshold for reaching destination
    int thresh = round(
        std::sqrt(
            std::pow(
                double(
                    currentNode.at(0).at(1).at(0) - endNode.at(0).at(1).at(0)),
                2)
                + std::pow(
                    double(
                        currentNode.at(0).at(1).at(1)
                            - endNode.at(0).at(1).at(1)),
                    2)));

    if (thresh <= 5) {
      path = backTrack(bT);
      return path;
    }
    /// Checking whether the destination is reached
    if (currentNode.at(0).at(1) == endNode.at(0).at(1)) {
      path = backTrack(bT);
      return path;
    }

    node.setTheta(currentNode.at(0).at(4).at(0));
    /// calling the motionModel() function
    std::vector<std::vector<double>> motion = motionModel();

    /// Exploring all the neighbor nodes
    for (auto newPos : motion) {
      /// setting the member variables of nodes through setters
      node.setCurrent(
          std::vector<double> { currentNode.at(0).at(1).at(0) + newPos.at(0),
              currentNode.at(0).at(1).at(1) + newPos.at(1) });
      /// setting the member variables of nodes through setters
      node.setGCost(currentNode.at(0).at(3).at(0) + newPos.at(2));
      /// setting the member variables of nodes through setters
      node.setHCost(
          double(
              std::sqrt(
                  std::pow(
                      endNode.at(0).at(1).at(0) - node.getCurrent()[0],
                      2)
                      + std::pow(endNode.at(0).at(1).at(1) -
                               node.getCurrent()[1], 2))));
      /// setting the member variables of nodes through setters
      node.setTotalCost(node.getHCost() + (1.0 * node.getGCost()));
      /// setting the member variables of nodes through setters
      node.setParent(currentNode.at(0).at(1));
      /// setting the member variables of nodes through setters
      node.setTheta(newPos[3]);

      double nodeVelocity = newPos[4];

      node.setDTheta(newPos[5]);

      /// checking if the explored node is within boundaries
      if (node.getCurrent()[0] > (1010 - 1) || node.getCurrent()[0] < 0
          || node.getCurrent()[1] > (1110 - 1) || node.getCurrent()[1] < 0) {
        continue;
      }

      ///  checking if the explored node is not hitting the obstacle
      if (costMap.at<double>(node.getCurrent()[0], node.getCurrent()[1]) != 1) {
        continue;
      }
      /// updating the costMap
      costMap.at<double>(node.getCurrent()[0], node.getCurrent()[1]) = 0;

      nestVector newNode;

      newNode.push_back(
          { { node.getTotalCost() }, node.getCurrent(), node.getParent(), { node
              .getGCost() }, { node.getTheta() }, { nodeVelocity }, { node
              .getDTheta() } });
      /// Adding the explored node to unvisited
      unvisited.push(
          { { { node.getTotalCost() }, node.getCurrent(), node.getParent(), {
              node.getGCost() }, { node.getTheta() }, { nodeVelocity }, { node
              .getDTheta() } } });
    }
  }
}


void AStar::rosTurtle(std::vector<std::vector<std::vector<double>>> rosInputs) {
  ros::NodeHandle n;
  /// Defining the publisher for cmd_vel topic
  ros::Publisher velPub = n.advertise < geometry_msgs::Twist
      > ("/cmd_vel", 1000);
  /// Declaring the object of geometry_msgs::Twist
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  /// setting the publishing rate to 10HZ
  ros::Rate loop_rate(10);

  int c = 0;
  int num = 0;
  while (ros::ok()) {
    if (c <= 10) {
      /// stop the robot if goal is reached
      if (num == rosInputs.size()) {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        velPub.publish(msg);
        break;
      }
      double x = (rosInputs.at(num).at(0).at(0)) / 100;
      double w = (rosInputs.at(num).at(1).at(0));
      /// Setting the linear and angular velocities
      msg.linear.x = x;
      msg.angular.z = w;
      /// Publishing the msg on topic
      velPub.publish(msg);
      if (c == 10) {
        num = num + 1;
        c = 0;
      }
    }
    c = c + 1;
    loop_rate.sleep();
    }
}

