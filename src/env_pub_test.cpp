//
// Created by lesley on 18-8-30.
//

#include <ros/ros.h>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <sstream>

#include <visualization_msgs/Marker.h>
#include <rrt_implement/world.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/point.h>
#include <rrt_implement/ellipsoid_points.h>
#include <rrt_implement/position.h>
#include <rrt_implement/pointsArray.h>
#include <geometry_msgs/Point.h>

//#include <rrt_implement/environment_publisher2.h>



void publish_world_info(const rrt_implement::world::ConstPtr& msg){
//  object_ellipsoids = msg->obstacles;
//  arena_ellipsoid = msg->arena;
//  robot_ellipsoid = msg->robot;
  ROS_INFO("succeed");
  std::cout<<msg->obstacle_size<<std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Environment_publisher");

  ros::NodeHandle nh;


  ros::Subscriber environment_info_sub = nh.subscribe("world_info", 1000, &publish_world_info);

//  getPoints();
//  plot();

  ros::spin();
  return 0;
}