//
// Created by lesley on 18-8-23.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

// MoveIt!
/*#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>*/

#include <visualization_msgs/Marker.h>
#include <rrt_implement/rrt.h>
//#include <rrt_implement/planner.h>

namespace rrt{
  class BasicMesh{
  private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    std::string file_;

    std::vector<geometry_msgs::Pose2D> poses_;
    int problem_id;

    visualization_msgs::Marker marker_;

  public:
    BasicMesh(ros::NodeHandle &nh, std::string file){
      nh_ = nh;

      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("agent", 1);

      file_ = file; //absolute dir

      marker_.header.frame_id = "base_link";
      marker_.ns = "rrt";
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sceneDisp");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  static const std::string PLANNING_GROUP = "base_link";
//  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
//  visual_tools.deleteAllMarkers();
  return 0;
}
