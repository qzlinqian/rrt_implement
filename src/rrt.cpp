//
// Created by Qian Lin on 18-8-21.
//

#include <rrt_implement/rrt.h>
#include <cmath>


//C-space is a 10*10 space
// 10 * 10 * 180(mod 180)
const double rrt::xRange = 10.0,
    rrt::yRange = 10.0,
    rrt::phiRange = 360.0; //the angle is stored in degree

const double rrt::xMetric = 0.1, //Metric of x
    rrt::yMetric = 0.1, //Metric of y
    rrt::phiMetric = phiRange/xRange*xMetric; //Metric of angle

const double rrt::MaxRange = 2*(xRange/xMetric + yRange/yMetric + phiRange/phiMetric);


//Position
rrt::Position::Position(double xx, double yy, double ph){
  x = xx;
  y = yy;
  phi = ph;
}

rrt::Position::Position(const rrt::Position &p) {
  x = p.x;
  y = p.y;
  phi = p.phi;
}


rrt::rrtNode::rrtNode(double xx, double yy, double ph, int ff) {
  x = xx;
  y = yy;
  phi = ph;
  father = ff;
  NodeID = -1;
}

//????? need to think through
rrt::rrtNode::rrtNode(rrt::rrtNode const &p) {
  x = p.x;
  y = p.y;
  phi = p.phi;
  father = p.father; //???
  NodeID = -1; //???
}

//father == 0: root; father == -1: not in tree; ID == -1: not in tree

//rrt::RRTree::RRTree() {
//    root = nullptr;
//}

std::vector<rrt::rrtNode> rrt::RRTree::rrtTree;

rrt::RRTree::RRTree(double originX, double originY, double originPhi) {
  rrtNode temp(originX, originY, originPhi, 0);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
//    root = &rrtTree.front();
}

rrt::RRTree::RRTree(rrt::Position const &originPos) {
  rrtNode temp(originPos.x, originPos.y, originPos.phi, 0);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
}

void rrt::RRTree::insert(double xx, double yy, double ph, IDNumber IntendedFather) { //know the father
  rrt::rrtNode temp(xx, yy, ph,IntendedFather);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
  rrtTree[IntendedFather].children.push_back(temp.NodeID);
}

void rrt::RRTree::insert(rrt::Position const &pos, rrt::IDNumber IntendedFather) {
  rrt::rrtNode temp(pos.x, pos.y, pos.phi);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
  rrtTree[IntendedFather].children.push_back(temp.NodeID);
}

rrt::rrtNode rrt::RRTree::remove(IDNumber ToBeRemoved) {
  rrt::rrtNode temp = rrtTree[ToBeRemoved];
  rrtTree.erase(rrtTree.begin()+ToBeRemoved);
  return temp;
}

rrt::rrtNode rrt::RRTree::pop() {
  rrtNode temp = rrtTree.back();
  //father wasn't copied by the copy constructor, so temp.father = -1
  rrtTree[rrtTree.back().father].children.pop_back(); //remove father's child
  rrtTree.pop_back();
  return temp;
}

std::vector<rrt::rrtNode>& rrt::RRTree::getTree() {
  return rrtTree;
}

//rrt::rrtNode& rrt::RRTree::getNode(IDNumber ID) {  //avoid copy constructor
//  return rrtTree[ID];
//}

double rrt::getEuclideanDistance(rrt::rrtNode const &Point1, rrt::Position const &Point2) {
  //avoid copy constructor
  double a = Point1.phi-Point2.phi;
  if (std::abs(a) > std::abs(360-a)) a = 360-a;
  return std::abs((Point1.x-Point2.x)/xMetric)
         +std::abs((Point1.y-Point2.y)/yMetric)
         +std::abs(a/phiMetric);
}

int rrt::RRTree::getTreeSize() {
  return int(rrtTree.size());
}

/*
rrt::rrtNode& rrt::RRTree::getTopNode() {
  return rrtTree.back();
}*/