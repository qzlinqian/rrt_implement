#include <ros/ros.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <rrt_implement/node.h>
#include <rrt_implement/rrtGraph.h>

class Graph_publisher{
private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber graph_sub;

  visualization_msgs::Marker marker_points;
  visualization_msgs::Marker marker_lines;

  double steps = 0.02;

public:
  explicit Graph_publisher(ros::NodeHandle &nh);
  ~Graph_publisher() = default;

  void publish_graph_info(const rrt_implement::rrtGraph::ConstPtr& msg);

  void plot();
};


Graph_publisher::Graph_publisher(ros::NodeHandle &nh) {
  nh_ = nh;

  graph_sub = nh_.subscribe("graph_info", 1000,
                                 &Graph_publisher::publish_graph_info, this);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("graph", 1);


  marker_points.header.frame_id = "/root";
  marker_points.ns = "graph points";
  marker_points.pose.orientation.w = 1.0;
  marker_points.id = 0;
  marker_points.type = visualization_msgs::Marker::POINTS;
  marker_points.scale.x = 0.06;
  marker_points.scale.y = 0.06;
  marker_points.color.a = 1.0f;
  marker_points.color.r = 0.8f;
  marker_points.color.b = 0.8f;
  marker_points.color.g = 0.0f;

  marker_lines.header.frame_id = "/root";
  marker_lines.ns = "graph lines";
  marker_lines.action = visualization_msgs::Marker::ADD;
  marker_lines.pose.orientation.w = 1.0;
  marker_lines.id = 1;
  marker_lines.type = visualization_msgs::Marker::LINE_LIST;
  marker_lines.scale.x = 0.05;
  marker_lines.color.a = 1.0f;
  marker_lines.color.r = 0.6f;
  marker_lines.color.g = 0.0f;
  marker_lines.color.b = 0.0f;
}


void Graph_publisher::publish_graph_info(const rrt_implement::rrtGraph::ConstPtr &msg) {
  unsigned long nodeNum = msg->graphNode.size();
//  unsigned long nodeNum = msg->graphNode.size()>20? 20: msg->graphNode.size();
  std::cout<<"Size:"<<nodeNum<<"\n";
//  std::cout<<"Msg received.\n"<<"Size: "<<msg->graphNode.size()<<"\n";

  std::vector<geometry_msgs::Point> points_;
  std::vector<geometry_msgs::Point> points2;

  points_.resize(2*nodeNum);
  points2.resize(nodeNum);

  while (ros::ok()){
    for(unsigned long k=1; k<nodeNum; k++){  //start from 0 will call the root whose father is -1
      rrt_implement::node temp = msg->graphNode[k];
      points2[k].x = temp.x;
      points2[k].y = temp.y;
      points2[k].z = 0.0;

      points_[2*k].x = temp.x;
      points_[2*k].y = temp.y;
      points_[2*k].z = 0.0;

      long temp_fatherID = temp.father;
      temp = msg->graphNode[temp_fatherID];
      points_[2*k+1].x = temp.x;
      points_[2*k+1].y = temp.y;
      points_[2*k+1].z = 0.0;
    }

    marker_points.points = points2;
    marker_points.header.stamp = ros::Time::now();
    marker_points.lifetime = ros::Duration();

    marker_lines.points = points_;
    marker_lines.header.stamp = ros::Time::now();
    marker_lines.lifetime = ros::Duration();

    while (marker_pub_.getNumSubscribers() < 1){
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub_.publish(marker_points);
    sleep(1);

    marker_pub_.publish(marker_lines);
//      std::cout<<"Plot graph.\n";
    sleep(2);
    ros::spinOnce();
  }
}


void Graph_publisher::plot() {

}

int main(int argc, char **argv){
  ros::init(argc, argv, "Graph_publisher");

  ros::NodeHandle n_h;
  Graph_publisher traj_pub(n_h);
  ros::spin();

  return 0;
}