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
#include <vector>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "AStar.hpp"




AStar::AStar() {
}

AStar::AStar(double rpm1Val, double rpm2Val, double dtVal) {
  rpm1 = rpm1Val;
  rpm2 = rpm2Val;
  dt = dtVal;
}

AStar::~AStar() {
}

typedef std::vector<std::vector<std::vector<double>>> nestVector;
nestVector AStar::backTrack(
		  std::vector<std::vector<std::vector<double>>> backList) {

//  for (int i = 0; i<backList.size(); i++) {
//    for (int j = 0; j<backList.at(i).size(); j++) {
//	for (int k = 0; k<backList.at(i).at(j).size(); k++) {
//    		std::cout << backList.at(i).at(j).at(k) << std::endl;
//    	}
//    }    
//  }
   
  std::vector<std::vector<double>> path;
  nestVector rosInputs;
  int length = backList.size();
  std::vector<double> currentPos = backList[length - 1][0];
  rosInputs.push_back( { backList[length - 1][2] ,
       backList[length - 1][3]  });
  path.push_back(currentPos);

  std::vector<double> parent = backList[length - 1][1];
  std::vector<double> zero;
  zero.push_back(0);
  zero.push_back(0);
  while (parent != zero) {
   for (int i = 0; i < length; i++) {
     std::vector<std::vector<double> > X = backList[i];
     if (X[0] == parent) {
       parent = X[1];
       currentPos = X[0];
       path.push_back(currentPos);
       rosInputs.push_back( { { X[2] }, { X[3] } });
     }
   } 
  } 
  std::reverse(path.begin(), path.end());
  std::reverse(rosInputs.begin(), rosInputs.end());
  return rosInputs;
}

    	


//std::vector<nestVector> AStar::backTrack(
//		  std::vector<std::vector<std::vector<double>>> n) {
//	std::vector<nestVector> temp1;
//	temp1.push_back(n);
//	return temp1;
//}

std::vector<std::vector<double>> AStar::motionModel() {
  double l = 35.4;
  double r = 3.8;
  std::vector<std::vector<double>> action;

  std::vector<std::array<double, 2>> steps { { 0.0, rpm1 }, { rpm1, 0.0 }, {
      rpm1, rpm1 }, { 0.0, rpm2 }, { rpm2, 0.0 }, { rpm2, rpm2 },
      { rpm1, rpm2 }, { rpm2, rpm1 } };

  double theta = node.getTheta();

  for (auto motion : steps) {
    double ur = (motion.at(0) * M_PI) / 30;

    double ul = (motion.at(1) * M_PI) / 30;
    double thetadot = (r / l) * (ur - ul);
    double dtheta = thetadot * dt;

    double change_theta = theta + dtheta;

    double xdot = (r / 2) * (ul + ur) * cos(change_theta);

    double ydot = (r / 2) * (ul + ur) * sin(change_theta);

    double dx = round(xdot * dt);
    double dy = round(ydot * dt);

    double cost = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));
    double vel_mag = std::sqrt(std::pow(xdot, 2) + std::pow(ydot, 2));

    action.push_back( { dx, dy, cost, change_theta, vel_mag, dtheta });
  }
    return action;
}


nestVector AStar::aStar(std::vector<double> startPoint, std::vector<double> goalPoint) {

  struct Compare {
    bool operator()(std::vector<std::vector<std::vector<double>>> const & a, std::vector<std::vector<std::vector<double>>> const & b)
    { return a[0][0][0] > b[0][0][0]; }
  };

  std::priority_queue< std::vector<std::vector<std::vector<double>>>, std::vector<std::vector<std::vector< std::vector<double> > >>, Compare > unvisited;
  std::priority_queue< std::vector<std::vector<std::vector<double>>>, std::vector<std::vector<std::vector< std::vector<double> > >>, Compare > visited;
  //double theta = node.getTheta();
  double theta = 0.0;

  nestVector startNode ;
  startNode.push_back({ {0.0}, startPoint, {0,0} , {0.0},
      {theta}, {0.0}, {0.0} });
  nestVector endNode ;
  endNode.push_back({ {0.0}, goalPoint, {0,0} , {0.0},
      {theta}, {0.0}, {0.0} });


//  std::make_heap(unvisited.begin(), unvisited.end());
//  unvisited.push_back(startNode);
//  push_heap(unvisited.begin(), unvisited.end());
  unvisited.push({{ {0.0}, startPoint, {0,0} , {0.0},
      {theta}, {0.0}, {0.0} }});
  
  ObstacleMap map(1110, 1010);
  map.setRadius(50.0);
//  cv::Mat costMap = map.createMap();
  cv::Mat costMap = cv::Mat::ones(1110, 1010, CV_64F);
  costMap.at<double>(int(startNode.at(0).at(1).at(0)), int(startNode.at(0).at(1).at(1))) = 0;
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::resizeWindow("Display frame", 505,555);
  cv::imshow("Display window", costMap);
  cv::waitKey(1);

  nestVector path;
  nestVector bT;
  while (unvisited.size() > 0) {
    nestVector currentNode;
    currentNode = unvisited.top();
//    std::cout << currentNode.at(0).at(0).at(0) << " " << currentNode.at(0).at(1).at(0) << " " << currentNode.at(0).at(1).at(1) << std::endl;
    
//    for (int i = 0; i<currentNode.at(0).size(); i++) {
//  	  for (int j = 0; j<currentNode.at(0).at(i).size(); j++) {
//  	  	std::cout << currentNode.at(0).at(i).at(j) << std::endl;
//  	  }
//    }    


    unvisited.pop();
    visited.push(currentNode);

    std::vector<int> temp;


    bT.push_back( { currentNode.at(0).at(1), currentNode.at(0).at(2), currentNode.at(0).at(5),
        currentNode.at(0).at(6) });

    int thresh = round(
        std::sqrt(
            std::pow(double(currentNode.at(0).at(1).at(0) - endNode.at(0).at(1).at(0)), 2)
                + std::pow(double(currentNode.at(0).at(1).at(1) - endNode.at(0).at(1).at(1)), 2)));


    if (thresh <= 5) {
      path = backTrack(bT);
      return path;

    }

    if (currentNode.at(0).at(1) == endNode.at(0).at(1)) {
      path = backTrack(bT);
      return path;
    }


    node.setTheta(currentNode.at(0).at(4).at(0));

    std::vector<std::vector<double>> motion = motionModel();
    //std::vector<double> neighbours;
    for (auto newPos : motion) {
      node.setCurrent(
          std::vector<double> { currentNode.at(0).at(1).at(0) + newPos.at(0), currentNode.at(0).at(1).at(1)
              + newPos.at(1) });
//      std::cout << node.getCurrent()[0] << " " << node.getCurrent()[1] << std::endl;  
      node.setGCost(double(currentNode.at(0).at(3).at(0)) + newPos.at(2));
      node.setHCost(
          double(
              std::sqrt(
                  std::pow(double(endNode.at(0).at(1).at(0) - node.getCurrent()[0]), 2)
                      + std::pow(double(endNode.at(0).at(1).at(1) - node.getCurrent()[1]),
                                 2))));
//      node.setTotalCost(double(node.getHCost() + node.getGCost()));

      node.setTotalCost(double(node.getHCost() + (1.0 * node.getGCost()) ));

      node.setParent( currentNode.at(0).at(1) );

      node.setTheta(newPos[3]);

      double nodeVelocity = newPos[4];

      node.setDTheta(newPos[5]);

      if (node.getCurrent()[0] > (1010 - 1) or node.getCurrent()[0] < 0
          or node.getCurrent()[1] > (1110 - 1) or node.getCurrent()[1] < 0) {
        continue;
      }

      if (costMap.at<double>(node.getCurrent()[0], node.getCurrent()[1]) != 1) {
        continue;
      }

      costMap.at<double>(node.getCurrent()[0], node.getCurrent()[1]) = 0;
      
//      cv::namedWindow("Display window", cv::WINDOW_NORMAL);
//      cv::resizeWindow("Display frame", 505,555);
//      cv::imshow("Display window", costMap);
//      cv::waitKey(10); 

      nestVector newNode;
      newNode.push_back({ {node.getTotalCost()}, node.getCurrent(), node
          .getParent(), {node.getGCost()}, {node.getTheta()}, {nodeVelocity}, {node
          .getDTheta()} });

      
//      unvisited.push_back(newNode);
      unvisited.push({{ {node.getTotalCost()}, node.getCurrent(), node
          .getParent(), {node.getGCost()}, {node.getTheta()}, {nodeVelocity}, {node
          .getDTheta()} }});
//      push_heap(unvisited.begin(), unvisited.end());

        
    
    }

  }

}

void AStar::rosTurtle(std::vector<std::vector<std::vector<double>>> rosInputs) {
  ros::NodeHandle n;
    
  ros::Publisher velPub = n.advertise < geometry_msgs::Twist > ("/cmd_vel", 1000);

  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  ros::Rate loop_rate(10);

  int c = 0;
  int num = 0;
  while (ros::ok()){
      if (c <= 10){
          if (num == rosInputs.size()){
              msg.linear.x = 0.0;
              msg.angular.z = 0.0;
              velPub.publish(msg);
              break;
           }
           double x = (rosInputs.at(num).at(0).at(0)) / 100;
           double w = (rosInputs.at(num).at(1).at(0));
           msg.linear.x = x ;
           msg.angular.z = w;   
           velPub.publish(msg);
           if(c == 10){
                //print(n, vel_bot.linear.x, vel_bot.angular.z)
               num = num + 1;
               c = 0;
           }
        }
        c = c+1 ;
        loop_rate.sleep();
    }
}

