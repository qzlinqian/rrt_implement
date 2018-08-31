//
// Created by lesley on 18-8-30.
//

#ifndef PROJECT_ENVIRONMENT_PUBLISHER2_H
#define PROJECT_ENVIRONMENT_PUBLISHER2_H

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


namespace rrt{
  void publish_world_info(const rrt_implement::world::ConstPtr& msg);
  void getPoints();
  void plotPoints(rrt_implement::pointsArray object, const ros::Publisher &publisher,
                  double r, double g, double b);
  void plot();

  ros::Publisher marker_robot_pub;
  ros::Publisher marker_arena_pub;
  std::vector<ros::Publisher> marker_obs_pubs;


  ros::ServiceClient point_gen_client;

  std::vector<rrt_implement::ellipsoid> object_ellipsoids;
  int object_size;
  rrt_implement::ellipsoid arena_ellipsoid;
  rrt_implement::ellipsoid robot_ellipsoid;

  std::vector<rrt_implement::pointsArray> collision_object_points;
  rrt_implement::pointsArray arena_points;
  rrt_implement::pointsArray robot_points;

  double steps = 0.1;
  int obs_num = 2;
}

#endif //PROJECT_ENVIRONMENT_PUBLISHER2_H
