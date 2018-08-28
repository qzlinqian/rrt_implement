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
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/world.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace rrt;
namespace ob = ompl::base;
namespace og = ompl::geometric;


PlanningAct::PlanningAct(Position &start, Position &goal):
    RRT_Tree(start) {
  startPos = start;
  goalPos = goal;
}


void rrt::setEnvironment(){
  robot_start.semi_axes.resize(2);
  robot_start.semi_axes[0]=2.0;
  robot_start.semi_axes[1]=1.0;
  robot_start.center.resize(2);
  robot_start.center[0]=-3.0;
  robot_start.center[1]=2.0;
  robot_start.angle=3.1416/2;
  robot_start.epsilon=2.0;

  arena.semi_axes.resize(2);
  arena.semi_axes[0] = 8.0;
  arena.semi_axes[1] = 6.0;
  arena.semi_axes[2] = 0.0;
  arena.angle = 0.0;
  arena.epsilon = 2.0;
  arena.center.resize(3);
  arena.center[0] = 0.0;
  arena.center[1] = 0.0;
  arena.center[2] = 0.0;

  arena_collision.semi_axes.resize(3);
  arena_collision.semi_axes[0] = 6.0;
  arena_collision.semi_axes[1] = 4.0;
  arena_collision.semi_axes[2] = 0.0;
  arena_collision.angle = 0.0;
  arena_collision.epsilon = 2.0;
  arena_collision.center.resize(3);
  arena_collision.center[0] = 0.0;
  arena_collision.center[1] = 0.0;
  arena_collision.center[2] = 0.0;

  obstacles.resize(2);
  rrt_implement::ellipsoid obs1;
  obs1.semi_axes.resize(3);
  obs1.semi_axes[0] = 2.0;
  obs1.semi_axes[1] = 1.0;
  obs1.semi_axes[2] = 0.0;
  obs1.angle = 3.1416/4;
  obs1.epsilon = 2.0;
  obs1.center.resize(3);
  obs1.center[0] = 0.0;
  obs1.center[1] = 1.0;
  obs1.center[2] = 0.0;

  rrt_implement::ellipsoid obs2;
  obs2.semi_axes.resize(3);
  obs2.semi_axes[0] = 3.0;
  obs2.semi_axes[1] = 2.0;
  obs2.semi_axes[2] = 0.0;
  obs2.angle = 3.1416/8;
  obs2.epsilon = 2.0;
  obs2.center.resize(3);
  obs2.center[0] = -3.0;
  obs2.center[1] = -3.0;
  obs2.center[2] = 0.0;

  obstacles[0] = obs1;
  obstacles[1] = obs2;

}

//need to complete
bool rrt::collisionChecking(rrt_implement::ellipsoid* robot_) {
  return true;
}

bool rrt::isStateValid(const Position new_position) {
  double x = new_position.x;
  double y = new_position.y;
  double yaw = new_position.phi;
  rrt_implement::ellipsoid robot;
  robot.semi_axes =robot_start.semi_axes;
  robot.epsilon = robot_start.epsilon;
  robot.center.resize(3);
  robot.center[0] = x;
  robot.center[1] = y;
  robot.center[2] = 0.0;
  robot.angle = yaw;

  bool valid = false;
  bool res = collisionChecking(&robot);
  if(res == false){
    valid = true;
  }
  std::cout <<"x: "<<x<<" y: "<< y<< " angle: "<< yaw<< std::endl;
  std::cout <<"Valid: "<< res<< std::endl;
  return valid;
}



Position rrt::generateRandomPos() {
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
//IDNumber PlanningAct::findNearestNode(Position randomPos) {
//Position PlanningAct::findNewPos() {
//bool PlanningAct::getNewNode() {
bool PlanningAct::planningWithRRT(){
//  bool planned = false;

  int searchTimes = 0;
  bool ends = false;  // or = goalIsAchieved(); in case goal==start?
  while (searchTimes < MaxSearchTimes && !ends) {
    IDNumber nearestNodeID = -1;
    Position randomPos = generateRandomPos();
    std::cout << "random node: x:" << randomPos.x << " y:" << randomPos.y << " phi:" << randomPos.phi << std::endl;

    double MinDistance = MaxRange, presentDistance;
    for (int i = 0; i < this->RRT_Tree.getTreeSize(); i++) {
      presentDistance = getEuclideanDistance(rrt::RRTree::rrtTree[i], randomPos);
      if (presentDistance < MinDistance) { //no action when "=" --lazy
        MinDistance = presentDistance;
        nearestNodeID = i;
      }
    }
    //nearestNodeID is the nearest Node to the random position in the tree
//  return nearestNodeID;
    std::cout << "q_near:" << nearestNodeID << std::endl;
    if (nearestNodeID > -1) { //find a new point from the nearest node towards the random position
      std::cout << "0 ";
      Position newPos;
      std::cout << "1 ";
      rrtNode nearestNode = rrt::RRTree::rrtTree[nearestNodeID];
      std::cout << "2 ";
      newPos.x = nearestNode.x + (randomPos.x - nearestNode.x) / MinDistance * xMetric;
      newPos.y = nearestNode.y + (randomPos.y - nearestNode.y) / MinDistance * yMetric;
      newPos.phi = nearestNode.phi + (randomPos.phi - nearestNode.phi) / MinDistance * phiMetric;
//    return newPos;
      if (isStateValid(newPos)) {  //can go through
        std::cout << "3 ";
        RRT_Tree.insert(newPos);
        std::cout << "4 ";
//      addedNewNode = true;
        ends = goalIsAchieved();
      }
    }
    searchTimes++;
  }

  if (ends){ //find trajectory
    unsigned long size = 0;  //trajectory tree size
    rrtNode tempNode = rrt::RRTree::rrtTree.back();
    int father = tempNode.father;

    while (father != -1){
      trajectory_[size].x = tempNode.x;
      trajectory_[size].y = tempNode.y;
      trajectory_[size].phi = tempNode.phi;
      size++;
      tempNode = rrt::RRTree::rrtTree[father];
      father = tempNode.father;
    }

    trajectory_.resize(size);
  }
  else
    std::cout<<"Planning Failed"<<std::endl;

  return /*planned*/ ends;
}

/*int PlanningAct::getTreeSize() {
  return RRT_Tree.getTreeSize();
}*/

//void PlanningAct::stepAMetricForward(rrtNode &nearestNode, Position &randomPos)

bool PlanningAct::goalIsAchieved() {
  return (getEuclideanDistance(rrt::RRTree::rrtTree.back(), goalPos) < NEAR_TO_GOAL);
}




int main(int argc, char **argv) {
  //init
  ros::init(argc, argv, "rrtPlanner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  world_pub = node_handle.advertise<rrt_implement::world>("world_info", 1);
  trajectory_pub = node_handle.advertise<rrt_implement::position>("trajectory_info",1);

//  client = node_handle.serviceClient<rrt_implement::collisionCheck>("collision_check_server");
  //world init & config
  setEnvironment();
  rrt_implement::world world_msg;
  world_msg.obstacles = obstacles;
  world_msg.obstacle_size = obstacles.size();
  world_msg.robot = robot_start;
  world_msg.arena = arena;

  //world publish
  while (world_pub.getNumSubscribers() < 1){
    ROS_WARN_ONCE("Please create a subscriber for the world_info");
    sleep(5);
  }
  world_pub.publish(world_msg);


  //init planner
  Position start_position(robot_start.center[0], robot_start.center[1], robot_start.angle), goal_position(3,3,0);
  PlanningAct planner(start_position, goal_position);

  bool planning_succeed = planner.planningWithRRT();

  if (planning_succeed){
    rrt_implement::trajectory trajectory_msg;
    trajectory_msg.points = trajectory_;
    trajectory_msg.model = robot_start;

    //trajectory publish
    while (trajectory_pub.getNumSubscribers() < 1){
      ROS_WARN_ONCE("Please create a subscriber for the world_info");
      sleep(1);
    }
    trajectory_pub.publish(trajectory_msg);
  }

  return 0;
}