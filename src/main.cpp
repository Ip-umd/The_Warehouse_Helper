#include <eigen3/Eigen/Core>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "AStar.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[]) {
 
  ros::init(argc, argv, "warehouseHelper");
  
  std::string startStation;
  std::string endStation;

  std::cout << "Enter the start station : " << std::endl;
  std::cin >> startStation;
   
  std::cout << "Enter the end station : " << std::endl;
  std::cin >> endStation;

  if(startStation == "A") {
  
    std::vector<double> Astp{520,900};
    std::vector<double> Aetp{700,450};
  
    std::vector<double> Bstp{700,450};
    std::vector<double> Betp{460,630};

    std::vector<double> Cstp{460,630};
    std::vector<double> Cetp{500,200};

    AStar objAstar(10,15,1);

    std::vector<std::vector<std::vector<double>>> Adiff = objAstar.aStar(Astp,Aetp);
    std::vector<std::vector<std::vector<double>>> Bdiff = objAstar.aStar(Bstp,Betp);
    std::vector<std::vector<std::vector<double>>> Cdiff = objAstar.aStar(Cstp,Cetp);
    int i = 0;
    while(i < 10) {
      Adiff.push_back({{0},{0.168617}});
      i = i + 1;  
    }
    Adiff.insert(std::end(Adiff), std::begin(Bdiff), std::end(Bdiff));

    int j = 0;
    while(j < 10) {
      Adiff.push_back({{0},{-0.4}});
      j = j + 1;  
    }

    Adiff.insert(std::end(Adiff), std::begin(Cdiff), std::end(Cdiff));

    int k = 0;
    while(k < 30) {
      Adiff.push_back({{0},{5.0}});
      k = k + 1;  
    }
	
    objAstar.rosTurtle(Adiff);
  }

  if(startStation == "B") {
    std::vector<double> Bstp{700,450};
    std::vector<double> Betp{460,630};

    std::vector<double> Cstp{460,630};
    std::vector<double> Cetp{500,200};

    AStar objAstar(10,15,1);

    std::vector<std::vector<std::vector<double>>> Bdiff = objAstar.aStar(Bstp,Betp);
    std::vector<std::vector<std::vector<double>>> Cdiff = objAstar.aStar(Cstp,Cetp);
    int j = 0;
    while(j < 10) {
      Bdiff.push_back({{0},{-0.4}});
      j = j + 1;  
    }

    Bdiff.insert(std::end(Bdiff), std::begin(Cdiff), std::end(Cdiff));

    int k = 0;
    while(k < 30) {
      Bdiff.push_back({{0},{5.0}});
      k = k + 1;  
    }
	
    objAstar.rosTurtle(Bdiff);
  }

  if(startStation == "C") {
    std::vector<double> Cstp{460,630};
    std::vector<double> Cetp{500,200};

    AStar objAstar(10,15,1);
    std::vector<std::vector<std::vector<double>>> Cdiff = objAstar.aStar(Cstp,Cetp);

    int k = 0;
    while(k < 30) {
      Cdiff.push_back({{0},{5.0}});
      k = k + 1;  
    }
	
    objAstar.rosTurtle(Cdiff);
  }



  return 0;
}
