//
// Created by lesley on 18-8-21.
//

//#include <cstdio>
//#include <ctime>
//#include <cstdlib>
#include <random>
#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>

rrt::PlanningAct::PlanningAct(rrt::Position &start, rrt::Position &goal):
    RRT_Tree(start) {
  startPos = start;
  goalPos = goal;
}

rrt::Position rrt::generateRandomPos() {
  Position randomPos;
  /*std::srand((unsigned)time(nullptr));
  randomNode.x = rand()/RAND_MAX * xRange; //get random x position
* it's not safe, use c++11 random library instead */
  //https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(0.0, 1.0);
  randomPos.x = dis(gen) * xRange;
  randomPos.y = dis(gen) * yRange;
  randomPos.phi = dis(gen) * phiRange;
  return randomPos;
}

//can only achieved by traverse all the node??
//rrt::IDNumber rrt::PlanningAct::findNearestNode(rrt::Position randomPos) {
//rrt::Position rrt::PlanningAct::findNewPos() {
bool rrt::PlanningAct::getNewNode() {
  IDNumber nearestNodeID = -1;
  Position randomPos = generateRandomPos();
  double MinDistance = MaxRange, presentDistance;
  for (int i=0; i < this->RRT_Tree.getTreeSize(); i++){
    presentDistance = getEuclideanDistance(RRT_Tree.getNode(i), randomPos);
    if (presentDistance < MinDistance){ //no action when "=" --lazy
      MinDistance = presentDistance;
      nearestNodeID = i;
    }
  }
  //nearestNodeID is the nearest Node to the random position in the tree
//  return nearestNodeID;
  if (nearestNodeID != -1){ //find a new point from the nearest node
    Position newPos;
    rrtNode nearestNode = RRT_Tree.getNode(nearestNodeID);
    newPos.x = nearestNode.x + (randomPos.x-nearestNode.x)/MinDistance * xMetric;
    newPos.y = nearestNode.y + (randomPos.y-nearestNode.y)/MinDistance * yMetric;
    newPos.phi = nearestNode.phi + (randomPos.phi-nearestNode.phi)/MinDistance * phiMetric;
//    return newPos;
    if (collisionChecking(nearestNode, newPos)) {  //can go through
      RRT_Tree.insert(newPos);
      return true;
    } else
      return false;
  } else
    return false;
}

/*int rrt::PlanningAct::getTreeSize() {
  return RRT_Tree.getTreeSize();
}*/

//void rrt::PlanningAct::stepAMetricForward(rrt::rrtNode &nearestNode, rrt::Position &randomPos)

bool rrt::PlanningAct::goalIsAchieved() {
  return (getEuclideanDistance(RRT_Tree.getTopNode(), goalPos) < NEAR_TO_GOAL);
}


//need to be completed
bool rrt::collisionChecking(rrt::rrtNode &Point1, rrt::Position &Point2) { return true;}