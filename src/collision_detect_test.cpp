//
// Created by lesley on 18-8-31.
//

#include "ros/ros.h"
#include <rrt_implement/world.h>
#include <rrt_implement/position.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/collision_detection.h>
#include <cstdlib>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipsoid_points_gen_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rrt_implement::collision_detection>("collision_detection");
  rrt_implement::collision_detection srv;
  rrt_implement::ellipsoid el_test;
  el_test.center = {0,0};
  el_test.angle = 0;
  el_test.semi_axes = {5,3};
  srv.request.environment.arena = el_test;
  srv.request.environment.robot = el_test;

  rrt_implement::position pos_test;
  pos_test.x=0;
  pos_test.y=0;
  pos_test.phi=0;
  srv.request.current_position = pos_test;

  if (client.call(srv))
  {
    ROS_INFO_STREAM("succeed");
    if (srv.response.collision) std::cout<<"collision\n";
    else std::cout<<"free\n";
  }
  else
  {
    ROS_ERROR("Failed to call service ellipsoid_points_gen");
    return 1;
  }

  return 0;
}

