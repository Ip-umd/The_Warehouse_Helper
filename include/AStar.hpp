#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "NodeParam.hpp"
#include "ObstacleMap.hpp"


class AStar {
 private: 
  
  NodeParam node;

  double rpm1;

  double rpm2;

  double dt;

 public:
  AStar();

  AStar(double rpm1Val, double rpm2Val, double dtVal);

  ~AStar();

  std::vector<std::vector<std::vector<double>>> backTrack(
		  std::vector<std::vector<std::vector<double>>> backList);

  std::vector<std::vector<double>> motionModel();

  std::vector<std::vector<std::vector<double>>> aStar(
      std::vector<double> startPoint, std::vector<double> goalPoint);

  void rosTurtle(std::vector<std::vector<std::vector<double>>> rosInputs);
};






