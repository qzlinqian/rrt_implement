//
// Created by Qian on 18-8-30.
//

#include <ros/ros.h>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include <visualization_msgs/Marker.h>
#include <rrt_implement/world.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/point.h>
#include <rrt_implement/ellipsoid_points.h>
#include <rrt_implement/position.h>
#include <rrt_implement/pointsArray.h>
#include <geometry_msgs/Point.h>

#include <rrt_implement/environment_publisher2.h>


using namespace rrt;

int main(int argc, char** argv){
  ros::init(argc, argv, "Environment_publisher");

  ros::NodeHandle nh;

  marker_robot_pub = nh.advertise<visualization_msgs::Marker>("robot", 1);
  marker_arena_pub = nh.advertise<visualization_msgs::Marker>("arena", 1);
  marker_obs_pubs.resize(static_cast<unsigned long>(obs_num));
  for(int i=0; i<marker_obs_pubs.size(); i++){
    std::ostringstream os;
    os<<i;
    marker_obs_pubs[i] = nh.advertise<visualization_msgs::Marker>("obstacle"+os.str(),1);
  }

  point_gen_client = nh.serviceClient<rrt_implement::ellipsoid_points>("ellipsoid_points_gen");

  ros::Subscriber environment_info_sub = nh.subscribe("world_info", 1000, publish_world_info);

//  getPoints();
//  plot();

  ros::spin();
  return 0;
}


void rrt::publish_world_info(const rrt_implement::world::ConstPtr& msg){
  object_ellipsoids = msg->obstacles;
  arena_ellipsoid = msg->arena;
  robot_ellipsoid = msg->robot;
}


void rrt::getPoints(){
  //from collision objects
  collision_object_points.resize(static_cast<unsigned long>(object_size));
  int i = 0;
  for (std::vector<rrt_implement::ellipsoid>::const_iterator j=object_ellipsoids.begin();
       j != object_ellipsoids.end(); j++){
    rrt_implement::ellipsoid ans = *j;
    rrt_implement::ellipsoid_points srv;
    srv.request.ellipsoid = ans;
    srv.request.steps = steps;

    if (point_gen_client.call(srv)){
      collision_object_points[i].name = "Obstacle"+i;
      collision_object_points[i].points = srv.response.points;
      i++;
    }
    else
      ROS_ERROR("Failed to call service ellipsoid_point_gen_server for obstacle");
  }

  //from robot
  rrt_implement::ellipsoid_points srv_robot;
  srv_robot.request.ellipsoid = robot_ellipsoid;
  srv_robot.request.steps = steps;
  if (point_gen_client.call(srv_robot)){
    robot_points.name = "Robot";
    robot_points.points = srv_robot.response.points;
  }
  else
    ROS_ERROR("Failed to call service ellipsoid_point_gen_server for robot");

  //from world
  rrt_implement::ellipsoid_points srv_arena;
  srv_arena.request.ellipsoid = arena_ellipsoid;
  srv_arena.request.steps = steps;
  if (point_gen_client.call(srv_arena)){
    arena_points.name = "Arena";
    arena_points.points= srv_arena.response.points;
  }
}


void rrt::plot() {
  while (ros::ok()){
    plotPoints(robot_points, marker_robot_pub,0.0,1.0,0.0);
    sleep(1);
    plotPoints(arena_points, marker_arena_pub,1.0,0.0,0.0);
    sleep(1);
    for(int i=0; i<collision_object_points.size(); i++){
      rrt_implement::pointsArray ans = collision_object_points[i];
      plotPoints(ans, marker_obs_pubs[i],0.0,0.0,1.0);
      sleep(1);
    }
  }
}


void rrt::plotPoints(rrt_implement::pointsArray object, const ros::Publisher &publisher,
                double r, double g, double b) {
  visualization_msgs::Marker marker_points;
  visualization_msgs::Marker marker_lines;

  marker_points.header.frame_id = "/root";
  marker_points.action = visualization_msgs::Marker::ADD;
  marker_points.pose.orientation.w = 1.0;
  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;
  marker_points.scale.x = 0.05;
  marker_points.scale.y = 0.05;
  marker_points.color.a = 1.0f;
  marker_points.color.r = 1.0;

  marker_lines.header.frame_id = "/root";
  marker_lines.action = visualization_msgs::Marker::ADD;
  marker_lines.pose.orientation.w = 1.0;
  marker_lines.type = visualization_msgs::Marker::LINE_STRIP;
  marker_lines.scale.x = 0.05;
  marker_lines.color.a = 1.0;
  marker_lines.color.g = static_cast<float>(g);
  marker_lines.color.r = static_cast<float>(r);
  marker_lines.color.b = static_cast<float>(b);


  std::vector<geometry_msgs::Point> points_;
  int i = 0;
  points_.resize(object.points.size());
  for (std::vector<rrt_implement::point>::const_iterator j=object.points.begin();
       j != object.points.end(); j++){
    rrt_implement::point ans = *j;
    points_[i].x = ans.x;
    points_[i].y = ans.y;
    points_[i].z = 0.0;
    i++;
  }

  marker_points.points = points_;
  marker_points.header.stamp = ros::Time::now();
  marker_points.lifetime = ros::Duration();

  //add the first point to the end -> form a circle
  points_.push_back(points_[0]);
  marker_lines.points = points_;
  marker_lines.header.stamp = ros::Time::now();
  marker_lines.lifetime = ros::Duration();

  while (publisher.getNumSubscribers() < 1){
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  publisher.publish(marker_points);
  sleep(1);
  publisher.publish(marker_lines);
}