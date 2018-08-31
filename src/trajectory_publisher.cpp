//
// Created by lesley on 18-8-29.
//

#include <ros/ros.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <rrt_implement/position.h>
#include <rrt_implement/trajectory.h>
#include <rrt_implement/pointsArray.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/ellipsoid_points.h>


class Trajectory_publisher{
private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber trajectory_sub;
  ros::ServiceClient point_gen_client;

  std::vector<rrt_implement::pointsArray> trajectory_objects_;
  rrt_implement::ellipsoid robot_model;

  visualization_msgs::Marker marker_points;
  visualization_msgs::Marker marker_lines;

  double steps = 0.02;

public:
  explicit Trajectory_publisher(ros::NodeHandle &nh);
  ~Trajectory_publisher() = default;

  void publish_trajectory_info(const rrt_implement::trajectory::ConstPtr& msg);

  void plot();
};


Trajectory_publisher::Trajectory_publisher(ros::NodeHandle &nh) {
  nh_ = nh;

  trajectory_sub = nh_.subscribe("trajectory_info", 1000,
      &Trajectory_publisher::publish_trajectory_info, this);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);

  point_gen_client = nh.serviceClient<rrt_implement::ellipsoid_points>("ellipsoid_points_gen");

  marker_points.header.frame_id = "/root";
  marker_points.ns = "trajectory points";
  marker_points.pose.orientation.w = 1.0;
  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;
  marker_points.scale.x = 0.05;
  marker_points.scale.y = 0.05;
  marker_points.color.a = 1.0f;
  marker_points.color.g = 0.5f;

  marker_lines.header.frame_id = "/root";
  marker_lines.ns = "lines";
  marker_lines.action = visualization_msgs::Marker::ADD;
  marker_lines.pose.orientation.w = 1.0;
  marker_lines.id = 1;
  marker_lines.type = visualization_msgs::Marker::LINE_STRIP;
  marker_lines.scale.x = 0.05;
  marker_lines.color.r = 0.5f;
  marker_lines.color.a = 1.0f;
}


void Trajectory_publisher::publish_trajectory_info(const rrt_implement::trajectory::ConstPtr &msg) {
  std::cout<<"Msg received.\n"<<"size: "<<msg->points.size();
  trajectory_objects_.resize(msg->points.size());

  robot_model = msg->model;

  int i = 0;
  for (std::vector<rrt_implement::position>::const_iterator j=msg->points.begin();
      j != msg->points.end(); j++){
    rrt_implement::position temp = *j;

    rrt_implement::ellipsoid_points srv;

    //get points
    robot_model.center[0] = temp.x;
    robot_model.center[1] = temp.y;
    robot_model.angle = temp.phi;
    srv.request.ellipsoid = robot_model;
    srv.request.steps = steps;

    if (point_gen_client.call(srv)){
      trajectory_objects_[i].name = "Robot" + i;
      trajectory_objects_[i].points = srv.response.points;
      i++;
    }
    else
      ROS_ERROR("Failed to call service ellipsoid_points_gen_server of trajectory");
  }
  plot();
}


void Trajectory_publisher::plot() {
  while (ros::ok()){
    for(int j = trajectory_objects_.size(); j>0; j--){
      rrt_implement::pointsArray obj_temp = trajectory_objects_[j-1];

      std::vector<geometry_msgs::Point> points_;
      points_.resize(obj_temp.points.size());
      for(int k=0; k<obj_temp.points.size(); k++){
        points_[k].x = obj_temp.points[k].x;
        points_[k].y = obj_temp.points[k].y;
        points_[k].z = 0.0;
      }

      marker_points.points = points_;
      marker_points.header.stamp = ros::Time::now();
      marker_points.lifetime = ros::Duration();

      //add the first point to the end -> form a circle
      points_.push_back(points_[0]);
      marker_lines.points = points_;
      marker_lines.header.stamp = ros::Time::now();
      marker_lines.lifetime = ros::Duration();

      while (marker_pub_.getNumSubscribers() < 1){
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub_.publish(marker_lines);
      std::cout<<"Plot trajectory.\n";
      sleep(2);
    }
    ros::spinOnce();
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Trajectory_publisher");

  ros::NodeHandle n_h;
  Trajectory_publisher traj_pub(n_h);
  ros::spin();

  return 0;
}