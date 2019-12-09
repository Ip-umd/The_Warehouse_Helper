#include "ObstacleMap.hpp"
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>


ObstacleMap::ObstacleMap() {
  l = 0;
  w = 0;
  robotRadius = 10.0;
}

ObstacleMap::ObstacleMap(int xval, int yval) {
  l = xval;
  w = yval;
}

ObstacleMap::~ObstacleMap() {
}

double ObstacleMap::getRadius() {
  return robotRadius;
}

void ObstacleMap::setRadius(double radius) {
  robotRadius = radius;
}

bool ObstacleMap::createRectangles(int x, int y) {
  int f1 = x - 918 - robotRadius;
  int f2 = 832 - x - robotRadius;
  int f3 = 827 - y - robotRadius;
  int f4 = y - 1070 - robotRadius;
  bool rec1 = (f1 <= 0 && f2 <= 0 && f3 <= 0 && f4 <= 0);

  int f1_2 = x - 1026 - robotRadius;
  int f2_2 = 983 - x - robotRadius;
  int f3_2 = 919 - y - robotRadius;
  int f4_2 = y - 1010 - robotRadius;
  bool rec2 = (f1_2 <= 0 && f2_2 <= 0 && f3_2 <= 0 && f4_2 <= 0);

  int f1_3 = x - 1110 - robotRadius;
  int f2_3 = 744 - x - robotRadius;
  int f3_3 = 621 - y - robotRadius;
  int f4_3 = y - 697 - robotRadius;
  bool rec3 = (f1_3 <= 0 && f2_3 <= 0 && f3_3 <= 0 && f4_3 <= 0);

  int f1_4 = x - 1110 - robotRadius;
  int f2_4 = 1052 - x - robotRadius;
  int f3_4 = 448 - y - robotRadius;
  int f4_4 = y - 565 - robotRadius;
  bool rec4 = (f1_4 <= 0 && f2_4 <= 0 && f3_4 <= 0 && f4_4 <= 0);

  int f1_5 = x - 1110 - robotRadius;
  int f2_5 = 1019 - x - robotRadius;
  int f3_5 = 362 - y - robotRadius;
  int f4_5 = y - 448 - robotRadius;
  bool rec5 = (f1_5 <= 0 && f2_5 <= 0 && f3_5 <= 0 && f4_5 <= 0);

  int f1_6 = x - 1110 - robotRadius;
  int f2_6 = 1052 - x - robotRadius;
  int f3_6 = 178 - y - robotRadius;
  int f4_6 = y - 295 - robotRadius;
  bool rec6 = (f1_6 <= 0 && f2_6 <= 0 && f3_6 <= 0 && f4_6 <= 0);

  int f1_7 = x - 1110 - robotRadius;
  int f2_7 = 927 - x - robotRadius;
  int f3_7 = 35 - y - robotRadius;
  int f4_7 = y - 111 - robotRadius;
  bool rec7 = (f1_7 <= 0 && f2_7 <= 0 && f3_7 <= 0 && f4_7 <= 0);

  int f1_8 = x - 1110 - robotRadius;
  int f2_8 = 685 - x - robotRadius;
  int f3_8 = 0 - y - robotRadius;
  int f4_8 = y - 35 - robotRadius;
  bool rec8 = (f1_8 <= 0 && f2_8 <= 0 && f3_8 <= 0 && f4_8 <= 0);

  int f1_9 = x - 896 - robotRadius;
  int f2_9 = 779 - x - robotRadius;
  int f3_9 = 35 - y - robotRadius;
  int f4_9 = y - 93 - robotRadius;
  bool rec9 = (f1_9 <= 0 && f2_9 <= 0 && f3_9 <= 0 && f4_9 <= 0);

  int f1_10 = x - 748 - robotRadius;
  int f2_10 = 474 - x - robotRadius;
  int f3_10 = 35 - y - robotRadius;
  int f4_10 = y - 187 - robotRadius;
  bool rec10 = (f1_10 <= 0 && f2_10 <= 0 && f3_10 <= 0 && f4_10 <= 0);

  int f1_11 = x - 712 - robotRadius;
  int f2_11 = 529 - x - robotRadius;
  int f3_11 = 265 - y - robotRadius;
  int f4_11 = y - 341 - robotRadius;
  bool rec11 = (f1_11 <= 0 && f2_11 <= 0 && f3_11 <= 0 && f4_11 <= 0);

  int f1_12 = x - 529 - robotRadius;
  int f2_12 = 438 - x - robotRadius;
  int f3_12 = 315 - y - robotRadius;
  int f4_12 = y - 498 - robotRadius;
  bool rec12 = (f1_12 <= 0 && f2_12 <= 0 && f3_12 <= 0 && f4_12 <= 0);

  int f1_13 = x - 936 - robotRadius;
  int f2_13 = 784 - x - robotRadius;
  int f3_13 = 267 - y - robotRadius;
  int f4_13 = y - 384 - robotRadius;
  bool rec13 = (f1_13 <= 0 && f2_13 <= 0 && f3_13 <= 0 && f4_13 <= 0);
  bool req = false;
  if (rec1 || rec2 || rec3 || rec4 || rec5 || rec6 || rec7 || rec8 || rec9
      || rec10 || rec11 || rec12 || rec13)
    req = true;
  return req;
}

bool ObstacleMap::createCircles(int x, int y) {
  int eqnCircle1 = std::pow((x - 390), 2) + std::pow((y - 965), 2) - std::pow((40.5 + robotRadius), 2);
  int eqnCircle2 = std::pow((x - 438), 2) + std::pow((y - 736), 2)
      - std::pow((40.5 + robotRadius), 2);
  int eqnCircle3 = std::pow((x - 390), 2) + std::pow((y - 45), 2)
      - std::pow((40.5 + robotRadius), 2);
  int eqnCircle4 = std::pow((x - 438), 2) + std::pow((y - 274), 2)
      - std::pow((40.5 + robotRadius), 2);
  bool req = false;

  if (eqnCircle1 <= 0 || eqnCircle2 <= 0 || eqnCircle3 <= 0 || eqnCircle4 <= 0)
    req = true;
  return req;
}

bool ObstacleMap::createTables(int x, int y) {
  int sq_1 = x - 310;
  int sq_2 = 150 - x;
  int sq_3 = 750 - y - robotRadius;
  int sq_4 = y - 910 - robotRadius;
  bool bool1 = (sq_1 <= 0 && sq_2 <= 0 && sq_3 <= 0 && sq_4 <= 0);

  int eq_circle_1 = std::pow((x - 150), 2) + std::pow((y - 830), 2)
      - std::pow((80 + robotRadius), 2);

  int eq_circle_2 = std::pow((x - 310), 2) + std::pow((y - 830), 2)
      - std::pow((80 + robotRadius), 2);
  bool req = false;
  if (bool1 || eq_circle_1 <= 0 || eq_circle_2 <= 0)
    req = true;
  return req;
}

bool ObstacleMap::drawBoundary(int x, int y) {
  bool bool1 = (x >= 0 && x <= robotRadius)
      || (x >= 1109 - robotRadius && x <= 1109);
  //std::cout << "check" << bool1;
  bool bool2 = (y >= 0 && y <= robotRadius)
      || (y >= 1009 - robotRadius && y <= 1009);
  //std::cout << "check" << bool2;
  bool req = false;
  if (bool1 || bool2)
    req = true;
  return req;
}



cv::Mat ObstacleMap::createMap() {
  std::vector<int> ox;
  std::vector<int> oy;
  //cv::Mat Z = cv::Mat(1110, 1010, CV_64F);
  cv::Mat Z = cv::Mat::ones(1110, 1010, CV_64F);

  //m = Eigen::MatrixXd::Zero();
  ObstacleMap obsMap(1110, 1010);


  for (int row = 0; row < l; row++) {
    for (int col = 0; col < w; col++) {
      bool req0 = obsMap.createRectangles(row, col);
      bool req1 = obsMap.createTables(row, col);
      bool req2 = obsMap.createCircles(row, col);
      bool req3 = obsMap.drawBoundary(row, col);
      //std::cout << "check" << req3;
//      std::cout << row << " ";
//      std::cout << col << std::endl;
      if (req0 || req1 || req2 || req3) {

        Z.at<double>(row, col) = 0;

        ox.push_back(row);

        oy.push_back(col);
      }
    }
  }
  cv::Mat dst;

  double angle = 90;   
//  cv::Size srcSz = Z.size();
//  cv::Size dstSz(srcSz.height, srcSz.width);

//  int len = std::max(Z.cols, Z.rows);
//  cv::Point2f center(len/2., len/2.);
//  cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
//  cv::warpAffine(Z, dst, rot_mat, dstSz);

//  cv::Point2f pt(Z.cols/2., Z.rows/2.);
//  cv::Mat rot_mat = cv::getRotationMatrix2D(pt, 180, 1.0);
//  cv::warpAffine(Z, dst, rot_mat, cv::Size(Z.rows, Z.rows));

  cv::flip(Z.t(), dst, 0);
  
//  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
//  cv::resizeWindow("Display frame", 505,555);
//  cv::imshow("Display window", dst);
//  cv::waitKey(0);
  
//  std::cout << dst.rows << " "<< dst.cols ;
//  std::cout << dst.size();
  return dst;
}

