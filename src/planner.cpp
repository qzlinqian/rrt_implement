//
// Created by lesley on 18-8-21.
//

//#include <cstdio>
//#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <random>
#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>
#include <rrt_implement/position.h>
#include <rrt_implement/trajectory.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

ros::Publisher pos_pub;


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
  std::uniform_real_distribution<double> dis(-1.0, 1.0);
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
  std::cout<<"random node: x:"<<randomPos.x<<" y:"<<randomPos.y<<" phi:"<<randomPos.phi<<std::endl;
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
  std::cout<<"q_near:"<<nearestNodeID<<std::endl;
  if (nearestNodeID > -1){ //find a new point from the nearest node
    std::cout<<"0 ";
    Position newPos;
    std::cout<<"1 ";
    rrtNode nearestNode = RRT_Tree.getNode(nearestNodeID);
    std::cout<<"2 ";
    newPos.x = nearestNode.x + (randomPos.x-nearestNode.x)/MinDistance * xMetric;
    newPos.y = nearestNode.y + (randomPos.y-nearestNode.y)/MinDistance * yMetric;
    newPos.phi = nearestNode.phi + (randomPos.phi-nearestNode.phi)/MinDistance * phiMetric;
//    return newPos;
//    if (collisionChecking(nearestNode, newPos)) {  //can go through
    std::cout<<"3 ";
    RRT_Tree.insert(newPos);
    std::cout<<"4 ";
      /*return true;
    } else
      return false;*/
    return true;
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
bool rrt::collisionChecking(rrt::rrtNode &Point1/*, rrt::Position &Point2*/, planning_scene::PlanningScenePtr &planning_scene) {
  bool res = false;

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.robot_state.is_diff = true;
//  planning_scene_msg.is_diff = true;
  planning_scene->getPlanningSceneMsg(planning_scene_msg);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  robot_state::RobotState& current_state_r = planning_scene->getCurrentStateNonConst();
  collision_request.group_name = PLANNING_GROUP;
  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();

  collision_request.contacts = true;
  collision_request.max_contacts = 100;

  std::vector<double> joint_values;

  const robot_model::JointModelGroup* joint_model_group = current_state_r.getJointModelGroup(PLANNING_GROUP);
  current_state_r.copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[0] = Point1.x;
  joint_values[1] = Point1.y;
  joint_values[2] = Point1.phi;  //???
  current_state_r.setJointGroupPositions(joint_model_group, joint_values);
  planning_scene->setCurrentState(current_state_r);

  collision_result.clear();

  planning_scene->checkCollision(collision_request, collision_result, current_state_r, acm);

  if (collision_result.collision)
    res = true;

  return res;
}


int main(int argc, char **argv) {
  //init
  ros::init(argc, argv, "rrtPlanner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  pos_pub = node_handle.advertise<rrt_implement::position>("TreeNode",1);
  rrt::Position start_state(7,-5,0), goal_state(-4,2,90);
  rrt::PlanningAct my_plan(start_state, goal_state);
  int times = 0;
  while ( times<MaxSearchTimes && (!my_plan.goalIsAchieved()) ){
    bool acted = my_plan.getNewNode();
    times++;
//    bool collide = rrt::collisionChecking(my_plan.RRT_Tree.getTopNode(),planning_scene);
    if (acted) {
      if (/*collide*/ false)
        my_plan.RRT_Tree.pop();
      else {
        std::cout << "NODE";
        std::cout<< my_plan.RRT_Tree.getTreeSize() << std::endl;
        rrt::rrtNode newAddedNode = my_plan.RRT_Tree.getTopNode();

        rrt_implement::position position_msg;
        position_msg.x = newAddedNode.x;
        position_msg.y = newAddedNode.y;
        position_msg.phi = newAddedNode.phi;
        std::cout << "X:" << position_msg.x << " Y:" << position_msg.y << " \\Phi:" << position_msg.phi << std::endl;
        pos_pub.publish(position_msg);
      }
    }
    sleep(1);
  }
}