//
// Created by Qian on 18-8-21.
//

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <vector>

//#include "geometry_msgs/Point.h"

//#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

//#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
//#include <geometry_msgs/Transform.h>
//#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define MaxSearchTimes 1000

int main(int argc, char **argv){
  //init
  ros::init(argc, argv, "rrtPlanner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  //MoveGroupInterface setup
  static const std::string PLANNING_GROUP = "base";
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  group.setWorkspace(-10,-10,-M_PI,10,10,M_PI);

  //PlanningSceneInterface setup, used only for illustration
  //look up the robot description on ROS parameter server ad construct a RobotModel to use
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const auto &robot_model = robot_model_loader.getModel();
  //create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
//  robot_state::RobotStatePtr robotStatePtr(new robot_state::RobotState(robot_model));
//  const robot_state::JointModelGroup* joint_model_group = robotStatePtr->getJointModelGroup(PLANNING_GROUP);
  //no ROS pluginlib

  //construct a PlanningScene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface/*(new planning_scene::PlanningScene(robotModel))*/;

  //set up trajectory displayer
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //display trajectory msg
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Get valid joints
  const std::vector<std::string> 	joints = group.getActiveJoints();
  for (const auto &aux : joints) {
    std::cout << "Active joint : " << aux << std::endl;
  }

  //collision object msg define
  moveit_msgs::CollisionObject collision_object;
  const std::string planning_frame = group.getPlanningFrame();
  std::cout<<"Planning frame: "<<planning_frame<<std::endl;

  collision_object.header.frame_id = planning_frame;

  //the id of the object is used to identify it
  collision_object.id = "box1";

  //define the box to be added to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;
  primitive.dimensions[1] = 2.2;
  primitive.dimensions[2] = 0.3;

  //define box pose
/*  geometry_msgs::Pose box_pos;
  box_pos.position.x = 1.6;
  box_pos.position.y = 0.3;
  box_pos.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.plane_poses.push_back(box_pos);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  //add the collision object into the world
  ROS_INFO_NAMED("rrt_implement", "Add an object to the world");
  planning_scene_interface.addCollisionObjects(collision_objects);*/


  //get valid joint -- not


  //visualization ???
/*  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(PLANNING_GROUP);
  visual_tools.deleteAllMarkers();*/

/*  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  //Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Sleep a little to allow time to startup rviz, etc..
  //This ensures that visual_tools.prompt() isn't lost in a sea of logs
  ros::Duration(10).sleep();

  // We can also use visual_tools to wait for user input
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
*/

//scene setting up ends

  //planning start
  group.startStateMonitor();
  group.setStartStateToCurrentState();

  //set start & goal state
  rrt::Position goal_state = rrt::generateRandomPos(), start_state = rrt::generateRandomPos();
  std::vector< double > joint_start = group.getRandomJointValues ();
  for(std::vector<double>::const_iterator it = joint_start.begin(); it != joint_start.end(); ++it){
    double aux = *it;
    std::cout << "El value : " << aux << std::endl;
  }

  joint_start.resize(3);
  robot_state::RobotState robot_state_start(robot_model);

  std::vector<double> joint_goal(3);
  joint_goal[0] = goal_state.x;
  joint_goal[1] = goal_state.y;
  joint_goal[2] = goal_state.phi;
  start_state.x = joint_start[0];
  start_state.y = joint_start[1];
  start_state.phi = joint_start[2];

  robot_state_start.setJointGroupPositions(PLANNING_GROUP, joint_start);
  group.setStartState(robot_state_start);
/*  start_state.x = group.getCurrentJointValues()[0];
  start_state.y = group.getCurrentJointValues()[1];
  start_state.phi = group.getCurrentRPY(PLANNING_GROUP)[0];*/

//  std::vector<double> joint_goal(3);
  group.setJointValueTarget("virtual_joint", joint_goal);

  //init planner
  rrt::PlanningAct planning_act_rrt(start_state, goal_state);

//  std::cout<<"planning start"<<std::endl;
  std::cout<<"start: x:"<<start_state.x<<" y:"<<start_state.y<<" phi:"<<start_state.phi<<std::endl
           <<"goal: x:"<<goal_state.x<<" y:"<<goal_state.y<<" phi:"<<goal_state.phi<<std::endl;

  int times = 0;
  while ( times<MaxSearchTimes && (!planning_act_rrt.goalIsAchieved()) ){
    bool acted = planning_act_rrt.getNewNode();
    times++;
//    bool collide = rrt::collisionChecking(planning_act_rrt.RRT_Tree.getTopNode(),planning_scene);
    if (acted) {
      if (/*collide*/ false)
        planning_act_rrt.RRT_Tree.pop();
      else {
        std::cout << "NODE";
        std::cout<< planning_act_rrt.RRT_Tree.getTreeSize() << std::endl;
        rrt::rrtNode newAddedNode = planning_act_rrt.RRT_Tree.getTopNode();
        geometry_msgs::Pose2D value;
        value.x = newAddedNode.x;
        value.y = newAddedNode.y;
        value.theta = newAddedNode.phi;
        std::cout << "X:" << value.x << " Y:" << value.y << " \\Phi:" << value.theta << std::endl;
      }
    }
    sleep(1);
  }


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode error_code = group.plan(my_plan);

  if (error_code.val == error_code.SUCCESS){
    trajectory_msgs::MultiDOFJointTrajectoryPoint points_plan[50];
    int k = 0;
    moveit_msgs::RobotTrajectory robotTraj = my_plan.trajectory_;
    trajectory_msgs::MultiDOFJointTrajectory multi_traj = robotTraj.multi_dof_joint_trajectory;
    for(std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator it = multi_traj.points.begin(); it != multi_traj.points.end(); ++it){
      points_plan[k] = *it;
      k++;
    }

    for(int iter=0; iter<k; iter++){
      std::cout << "ITER:" << iter << std::endl;
      geometry_msgs::Vector3 value = points_plan[iter].transforms.at(0).translation;
      geometry_msgs::Quaternion quat = points_plan[iter].transforms.at(0).rotation;


      std::cout<< "X: " << value.x << ", " << "Y: " << value.y << ", " << "Z: " << quat.z << std::endl;

    }

  }


  sleep(10);

  ros::spin();
  return 0;
}