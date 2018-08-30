#include "ros/ros.h"
#include <rrt_implement/ellipsoid_points.h>
#include <rrt_implement/point.h>
#include <rrt_implement/ellipsoid.h>
#include <cstdlib>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipsoid_points_gen_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rrt_implement::ellipsoid_points>("ellipsoid_points_gen");
  rrt_implement::ellipsoid_points srv;
  rrt_implement::ellipsoid el_test;
  el_test.center = {0,0};
  el_test.angle = 0;
  el_test.semi_axes = {5,3};
  srv.request.ellipsoid = el_test;
  srv.request.steps = 0.1;

  if (client.call(srv))
  {
    ROS_INFO("succeed");
    for (int i=0; i<srv.response.points.size(); i++){
//      std::ostringstream os;
      std::cout<<"x:"<<srv.response.points[i].x<<" y:"<<srv.response.points[i].y<<std::endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service ellipsoid_points_gen");
    return 1;
  }

  return 0;
}