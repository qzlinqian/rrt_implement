//
// Created by Qian on 18-8-31.
//


/* Reference to:
 * https://stackoverflow.com/questions/12191060/figuring-out-ellipsoid-ellipsoid-collision-detection
 * simple but sometimes fails
float EllipsoidRadius(VECTOR ellipsoidRadius, VECTOR direction)
{
  direction /= ellipsoidRadius;
  normalize(direction);
  return length(direction * ellipsoidRadius);
  // The * is just a multiplication
}

bool EllipsoidIntersects(POINT positionA, VECTOR sizeA, POINT positionB, VECTOR sizeB)
{
  VECTOR direction = positionA - positionB;
  float distance = length(direction);

  float radiusA = EllipsoidRadius(sizeA, direction);
  float radiusB = EllipsoidRadius(sizeB, direction);

  return distance < radiusA + radiusB;
}*/

#include <ros/ros.h>
#include <rrt_implement/world.h>
#include <rrt_implement/position.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/collision_detection.h>

#include <cmath>

namespace rrt{
  double ellipsoidRadius(double a, double b, double angle){ //angle: [0,pi)
    return (a * std::abs(std::cos(angle)) + b * std::abs(std::sin(angle)) );
  }

  bool free_outside(const rrt_implement::ellipsoid &robot, const rrt_implement::ellipsoid &object){
    double deltaY = robot.center[1]-object.center[1];
    double deltaX = robot.center[0]-object.center[0];
    double inter_angle = std::atan( deltaY / deltaX );
    double robot_radius = ellipsoidRadius(robot.semi_axes[0], robot.semi_axes[1], robot.angle+inter_angle);
    double object_radius = ellipsoidRadius(object.semi_axes[0], object.semi_axes[1], object.angle+inter_angle);

    return ((std::abs(deltaX)+std::abs(deltaY))/1.07 > (robot_radius + object_radius));   // R+r < EuclideanDistance / sqrt(2) <= D
  }

  bool free_inside(const rrt_implement::ellipsoid &robot, const rrt_implement::ellipsoid &object){
    double deltaY = robot.center[1]-object.center[1];
    double deltaX = robot.center[0]-object.center[0];
    double inter_angle = std::atan( deltaY / deltaX );
    double robot_radius = ellipsoidRadius(robot.semi_axes[0], robot.semi_axes[1], robot.angle+inter_angle);
    double object_radius = ellipsoidRadius(object.semi_axes[0], object.semi_axes[1], object.angle+inter_angle);
//    return (std::sqrt((deltaX * deltaX)+(deltaY * deltaY)) < (object_radius - robot_radius));       // R-r > EuclideanDistance > D
    return ((std::abs(deltaX)+std::abs(deltaY))*1.1 < (object_radius - robot_radius));
  }

  bool isStateInCollision(rrt_implement::collision_detectionRequest &req, rrt_implement::collision_detectionResponse &res){
    res.collision = true;
    rrt_implement::ellipsoid current_robot= req.environment.robot;
//    current_robot.semi_axes[0] = req.environment.robot.semi_axes[0];   // don't visit req's member's member's member
//    current_robot.semi_axes[1] = req.environment.robot.semi_axes[1];
    current_robot.angle = req.current_position.phi;  // should not add the initial one, because the planner is based on the first pose
    current_robot.center[0] = req.current_position.x;
    current_robot.center[1] = req.current_position.y;
    if ( !free_inside(current_robot, req.environment.arena) )
      return true; //no more compute
    for (int i=0; i<req.environment.obstacle_size; i++){
      if ( !free_outside(current_robot, req.environment.obstacles[i]) )
        return true;
    }
    res.collision = false;
    return true;
  }
}

using namespace rrt;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_checking_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("collision_detection", isStateInCollision);
  ROS_INFO("Ready to check collision of current state.");
  ros::spin();

  return 0;
}

