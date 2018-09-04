//
// Created by lesley on 18-9-4.
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
#include <rrt_implement/rrtGraph.h>

using namespace rrt;



int main(){
  for (int i =0; i<20; i++) {
    newPos = generateRandomPos();
    //insert in graph
    rrt_implement::node temp_node;
    temp_node.x = newPos.x;
    temp_node.y = newPos.y;
    temp_node.father = nearestNodeID;

    graph_msg.graphNode.push_back(temp_node);
  }
}
