//
// Created by lesley on 18-8-21.
//

#include <cstdlib>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <random>
#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>
#include <rrt_implement/position.h>
#include <rrt_implement/trajectory.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/world.h>
#include <rrt_implement/collision_detection.h>

using namespace rrt;


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
  robot_start.angle=PI/2;

  arena.semi_axes.resize(2);
  arena.semi_axes[0] = 8.0;
  arena.semi_axes[1] = 6.0;
  arena.semi_axes[2] = 0.0;
  arena.angle = 0.0;
  arena.center.resize(3);
  arena.center[0] = 0.0;
  arena.center[1] = 0.0;
  arena.center[2] = 0.0;


  obstacles.resize(2);
  rrt_implement::ellipsoid obs1;
  obs1.semi_axes.resize(3);
  obs1.semi_axes[0] = 2.0;
  obs1.semi_axes[1] = 1.0;
  obs1.semi_axes[2] = 0.0;
  obs1.angle = PI/4;
  obs1.center.resize(3);
  obs1.center[0] = 0.0;
  obs1.center[1] = 1.0;
  obs1.center[2] = 0.0;

  rrt_implement::ellipsoid obs2;
  obs2.semi_axes.resize(3);
  obs2.semi_axes[0] = 3.0;
  obs2.semi_axes[1] = 2.0;
  obs2.semi_axes[2] = 0.0;
  obs2.angle = PI/8;
  obs2.center.resize(3);
  obs2.center[0] = -3.0;
  obs2.center[1] = -3.0;
  obs2.center[2] = 0.0;

  obstacles[0] = obs1;
  obstacles[1] = obs2;

}



bool rrt::isStateValid(const Position& new_position) {
  rrt_implement::position current_position_msg;
  current_position_msg.x = new_position.x;
  current_position_msg.y = new_position.y;
  current_position_msg.phi = new_position.phi / Degree2Radian;

  rrt_implement::collision_detection srv;
  srv.request.current_position = current_position_msg;
  srv.request.environment = world_msg;

  if (client.call(srv))
  {
    return (!srv.response.collision);
  }
  else
  {
    ROS_ERROR("Failed to call service collision_detection");
    return false;
  }
}



Position rrt::generateRandomPos() {
  Position randomPos;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-1.0, 1.0);
  randomPos.x = dis(gen) * xRange;
  randomPos.y = dis(gen) * yRange;
  randomPos.phi = dis(gen) * phiRange;
  return randomPos;
}



bool PlanningAct::planningWithRRT(){
  int searchTimes = 0;

  bool ends = false;  // or = goalIsAchieved(); in case goal==start?
  while (searchTimes < MaxSearchTimes && !ends) {
    IDNumber nearestNodeID = -1;
    Position randomPos = generateRandomPos();
//    std::cout << "random node: x:" << randomPos.x << " y:" << randomPos.y << " phi:" << randomPos.phi << std::endl;

    double MinDistance = MaxRange, presentDistance;
    for (int i = 0; i < this->RRT_Tree.rrtTree.size(); i++) {
      presentDistance = getEuclideanDistance(RRT_Tree.rrtTree[i], randomPos);
      if (presentDistance < MinDistance) { //no action when "=" --lazy
        MinDistance = presentDistance;
        nearestNodeID = i;
//        std::cout<<" searched: "<<i;
      }
    }
    //nearestNodeID is the nearest Node to the random position in the tree

    //std::cout << "q_near:" << nearestNodeID <<" dis"<<MinDistance<<" total:"<<RRT_Tree.rrtTree.size()<< std::endl;
    if (nearestNodeID > -1) { //find a new point from the nearest node towards the random position
      Position newPos;
      rrtNode nearestNode = RRT_Tree.rrtTree[nearestNodeID];
      newPos.x = nearestNode.x + (randomPos.x - nearestNode.x) / MinDistance * xMetric;
      newPos.y = nearestNode.y + (randomPos.y - nearestNode.y) / MinDistance * yMetric;
      double temp_phi = nearestNode.phi + (randomPos.phi - nearestNode.phi) / MinDistance * phiMetric;
      while (temp_phi > phiRange) temp_phi -= phiRange;
      while (temp_phi < 0) temp_phi += phiRange;
      newPos.phi = temp_phi;
      if (isStateValid(newPos)) {  //can go through
        RRT_Tree.insert(newPos, nearestNodeID);
        ends = goalIsAchieved();
      }
    }
    searchTimes++;
    if (searchTimes % 500 == 0) std::cout<<"Have searched for "<<searchTimes<<" times.\n Succeed for "<<RRT_Tree.rrtTree.size()<<" times.\n";
  }

  std::cout<<"Search ends in "<<searchTimes<<" times\n";


  if (ends){ //find trajectory
    rrtNode tempNode = RRT_Tree.rrtTree.back();
    int father = tempNode.father;

    rrt_implement::position temp_traj;

    while (father > -1){
      temp_traj.x = tempNode.x;
      temp_traj.y = tempNode.y;
      temp_traj.phi = tempNode.phi * Degree2Radian;
      trajectory_.push_back(temp_traj); //reverse order!!!
      tempNode = RRT_Tree.rrtTree[father];
      father = tempNode.father;
    }
    std::cout<<"Planning succeed and start to plot trajectory.\nTraj size:"<<trajectory_.size()<<"\n";
  }
  else
    ROS_INFO("Planning failed, too much times.");

  return ends;
}

//void PlanningAct::stepAMetricForward(rrtNode &nearestNode, Position &randomPos)

bool PlanningAct::goalIsAchieved() {
  return (getEuclideanDistance(RRT_Tree.rrtTree.back(), goalPos) < NEAR_TO_GOAL);
}




int main(int argc, char **argv) {
  //init
  ros::init(argc, argv, "rrtPlanner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  world_pub = node_handle.advertise<rrt_implement::world>("world_info", 1000);
  trajectory_pub = node_handle.advertise<rrt_implement::trajectory>("trajectory_info",1000);


  client = node_handle.serviceClient<rrt_implement::collision_detection>("collision_detection");
  //world init & config
  setEnvironment();
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
  ROS_INFO("world_info published");

  double temp_phi = robot_start.angle /Degree2Radian;


  //init planner
  Position start_position(robot_start.center[0], robot_start.center[1], temp_phi), goal_position(3,3,0);
  PlanningAct planner(start_position, goal_position);

  bool planning_succeed = planner.planningWithRRT();

  if (planning_succeed){
    rrt_implement::trajectory trajectory_msg;
    trajectory_msg.points = trajectory_;

    trajectory_msg.model = robot_start;

    //trajectory publish
    while (trajectory_pub.getNumSubscribers() < 1){
      ROS_WARN_ONCE("Please create a subscriber for the trajectory_info");
      sleep(1);
    }
    trajectory_pub.publish(trajectory_msg);
  }

  return 0;
}