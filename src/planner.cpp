//
// Created by lesley on 18-8-21.
//

//#include <cstdio>
//#include <ctime>
//#include <cstdlib>
#include <random>
#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>

rrt::rrtPlanner::rrtPlanner(rrt::rrtNode &start, rrt::rrtNode &goal):
    RRT_Tree(start.x, start.y, start.phi) {
  startPos = start;
  goalPos = goal;
}

rrt::rrtNode rrt::rrtPlanner::generateRandomNode() {
  rrtNode randomNode;
  /*std::srand((unsigned)time(nullptr));
  randomNode.x = rand()/RAND_MAX * xRange; //get random x position
* it's not safe, use c++11 random library instead */
  //reference to https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(0.0, 1.0);
  randomNode.x = dis(gen) * xRange;
  randomNode.y = dis(gen) * yRange;
  randomNode.phi = dis(gen) * phiRange;
  return randomNode;
}

//can only achieved by traverse all the node??
int rrt::rrtPlanner::findNearestNode(const rrt::rrtNode &inputNode) {
  int nearestNodeID = -1;
  double MinDistance = MaxRange, presentDistance;
  for (int i=0; i < this->RRT_Tree.getTreeSize(); i++){
    presentDistance = getEuclideanDistance(RRT_Tree.getNode(i), inputNode);
    if (presentDistance < MinDistance){ //no action when "=" --lazy
      presentDistance = MinDistance;
      nearestNodeID = i;
    }
  }
  return nearestNodeID;
}