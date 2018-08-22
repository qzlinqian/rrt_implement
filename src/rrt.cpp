//
// Created by Qian Lin on 18-8-21.
//

#include <rrt_implement/rrt.h>
#include <cmath>


//C-space is a 10*10 space
// 10 * 10 * 180(mod 180)
const double rrt::xRange = 10.0,
    rrt::yRange = 10.0,
    rrt::phiRange = 180.0; //the angle is stored in degree

const double rrt::xMetric = 0.1, //Metric of x
    rrt::yMetric = 0.1, //Metric of y
    rrt::phiMetric = 1.8; //Metric of angle

const double rrt::MaxRange = xRange/xMetric + yRange/yMetric + phiRange/phiMetric;

rrt::rrtNode::rrtNode(double xx, double yy, double ph, int ff) {
  x = xx;
  y = yy;
  phi = ph;
  father = ff;
  NodeID = -1;
}

//????? need to think through
rrt::rrtNode::rrtNode(const rrt::rrtNode &p) {
  x = p.x;
  y = p.y;
  phi = p.phi;
  father = -1; //???
  NodeID = -1; //???
}

//father == 0: root; father == -1: not in tree; ID == -1: not in tree

//rrt::RRTree::RRTree() {
//    root = nullptr;
//}

rrt::RRTree::RRTree(double originX, double originY, double originPhi) {
  rrtNode temp(originX, originY, originPhi, 0);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
//    root = &rrtTree.front();
}

void rrt::RRTree::insert(int IntendedFather, double xx, double yy, double ph) { //know the father
  rrt::rrtNode temp(xx, yy, ph,IntendedFather);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
  rrtTree[IntendedFather].children.push_back(temp.NodeID);
}

void rrt::RRTree::insert(double xx, double yy, double ph) { //not know the father
  rrt::rrtNode temp(xx, yy, ph);
  temp.NodeID = int(rrtTree.size());
  rrtTree.push_back(temp);
}

rrt::rrtNode rrt::RRTree::remove(int ToBeRemoved) {
  rrt::rrtNode temp = rrtTree[ToBeRemoved];
  rrtTree.erase(rrtTree.begin()+ToBeRemoved);
  return temp;
}

std::vector<rrt::rrtNode>& rrt::RRTree::getTree() {
  return rrtTree;
}

rrt::rrtNode& rrt::RRTree::getNode(int ID) {  //avoid copy constructor
  return rrtTree[ID];
}

double rrt::getEuclideanDistance(rrt::rrtNode const &Point1, rrt::rrtNode const &Point2) {
  //avoid copy constructor
  return std::abs((Point1.x-Point2.x)/xMetric)
         +std::abs((Point1.y-Point2.y)/yMetric)
         +std::abs((Point1.phi-Point2.phi)/phiMetric);
}

int rrt::RRTree::getTreeSize() {
  return int(rrtTree.size());
}