//
// Created by Qian on 18-8-21.
//

#include <ros/ros.h>
#include <cstdlib>
#include <vector>

#include "geometry_msgs/Point.h"

#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>

int main(int argc, char **argv){
  //init
  ros::init(argc, argv, "rrt_implement");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  //MoveGroupInterface & PlanningSceneInterface setup
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
  robot_model::RobotModelPtr robotModel = robotModelLoader.getModel(); //const?
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface/*(new planning_scene::PlanningScene(robotModel))*/;
  moveit::planning_interface::MoveGroupInterface group("base_link");

  //collision object define with msg
  moveit_msgs::CollisionObject collisionObject;
  std::string planningFrame = group.getPlanningFrame();
  std::cout<<planningFrame<<std::endl;

  collisionObject.header.frame_id = planningFrame;

  //the id of the object is used to identify it
  collisionObject.id = "box1";

  //define the box to be added to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;
  primitive.dimensions[1] = 2.2;
  primitive.dimensions[2] = 0.3;

  //define box pose
  geometry_msgs::Pose box_pos;
  box_pos.position.x = 1.6;
  box_pos.position.y = 0.3;
  box_pos.position.z = 0;

  collisionObject.primitives.push_back(primitive);
  collisionObject.plane_poses.push_back(box_pos);
  collisionObject.operation = collisionObject.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collisionObject);

  //add the collision object into the world
  ROS_INFO_NAMED("rrt_implement", "Add an object to the world");
  planningSceneInterface.addCollisionObjects(collision_objects);

  //set up trajectory displayer
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  //display trajectory msg
  moveit_msgs::DisplayTrajectory displayTrajectory;

  //get valid joint -- not

  sleep(10);

  ros::spin();
  return 0;
}