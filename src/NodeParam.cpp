#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "NodeParam.hpp"

NodeParam::NodeParam() {
}

NodeParam::~NodeParam() {

}

std::vector<double> NodeParam::getCurrent() {
  return currentNode;
}

void NodeParam::setCurrent(std::vector<double> current) {
  currentNode = current;
}

std::vector<double> NodeParam::getParent() {
  return parentNode;
}

void NodeParam::setParent(std::vector<double> parent) {
  parentNode = parent;
}

double NodeParam::getGCost() {
  return gCost;
}

void NodeParam::setGCost(double costToGo) {
  gCost = costToGo;
}

double NodeParam::getHCost() {
  return hCost;
}

void NodeParam::setHCost(double costToCome) {
  hCost = costToCome;
}

double NodeParam::getTotalCost() {
  return totalCost;
}

void NodeParam::setTotalCost(double total) {
  totalCost = total;
}

double NodeParam::getTheta() {
  return theta;
}

void NodeParam::setTheta(double th) {
  theta = th;
}

double NodeParam::getDTheta() {
  return dtheta;
}

void NodeParam::setDTheta(double dth) {
  dtheta = dth;
}

double NodeParam::getDx() {
  return dx;
}

void NodeParam::setDx(double dX) {
  dx = dX;
}

double NodeParam::getDy() {
  return dy;
}

void NodeParam::setDy(double dY) {
  dy = dY;
}
