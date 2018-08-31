//
// Created by lesley on 18-8-29.
//

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <utility>
#include <vector>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/point.h>
#include <rrt_implement/pointsArray.h>
#include <rrt_implement/ellipsoid_points.h>


/*rrt_implement::point*/ void rotation(double x_, double y_, double angle_, double* newPoint) {
//  rrt_implement::point newPoint;
  newPoint[0] = x_*std::cos(angle_) + y_*std::sin(angle_);
  newPoint[1] = y_*std::cos(angle_) - x_*std::sin(angle_);
//  return newPoint;
}


bool handle_ellipsoid_points_gen(
    rrt_implement::ellipsoid_points::Request &req, rrt_implement::ellipsoid_points::Response &res) {

  std::vector<double> semi_axes = req.ellipsoid.semi_axes, center = req.ellipsoid.center;
  double angle = req.ellipsoid.angle;

//  double n = req.ellipsoid.epsilon; //(0,2]

  double pi_step = M_PI * req.steps;
  auto pointsNum = static_cast<int>(std::ceil(2 / req.steps));

  res.points.resize(static_cast<unsigned long>(pointsNum));

  for (int i =0;i<pointsNum;i++){
    double x = semi_axes[0] * std::cos(i * pi_step);
    double y = semi_axes[1] * std::sin(i * pi_step);

    double temp[2];
    rotation(x,y,angle,temp);
    res.points[i].x = center[0] + temp[0];
    res.points[i].y = center[1] + temp[1];
//    std::cout<<i<<std::endl;
  }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipsoid_points_gen_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ellipsoid_points_gen", handle_ellipsoid_points_gen);
  ROS_INFO("Ready to generate ellipsoid points.");
  ros::spin();

  return 0;
}

