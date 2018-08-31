// to implement the rrt planner based on the configuration in rrt.h & rrt.cpp
// Created by Qian on 18-8-21.
//

#ifndef PROJECT_PLANNER_H
#define PROJECT_PLANNER_H

#include <string>
#include <rrt_implement/rrt.h>
#include <moveit/planning_scene/planning_scene.h>

#include <rrt_implement/position.h>
#include <rrt_implement/trajectory.h>
#include <rrt_implement/ellipsoid.h>
#include <rrt_implement/world.h>

#define PI 3.1416

#define NEAR_TO_GOAL 2

static std::string PLANNING_GROUP = "base";

#define MaxSearchTimes 6000

namespace rrt{
  class PlanningAct{
  private:
    Position startPos;
    Position goalPos;
  public:
    RRTree RRT_Tree;

    PlanningAct(Position &start, Position &goal);
    PlanningAct(){}

//    friend Position generateRandomPos();
//    IDNumber findNearestNode(rrt::Position randomPos);
//    void stepAMetricForward(rrtNode &nearestNode, Position& randomPos);  combine the two to reduce parameter pass
//    Position findNewPos(); followed with insert, combine again
    /*find the nearest node to the random point in the tree,
     * then  calculate the position of the point that generated by stepping a metric long
     * from the nearest point towards the random point*/
//    bool getNewNode(); followed with check whether goal is achieved, combine again
    bool planningWithRRT();
    bool goalIsAchieved();
//    bool collisionChecking(rrtNode &Point1, Position &Point2);
//    int getTreeSize();
  };

  Position generateRandomPos();
//  bool collisionChecking(rrtNode &Point1,/* Position &Point2, */planning_scene::PlanningScenePtr &planning_scene);
  bool collisionChecking(rrt_implement::ellipsoid* robot);

  bool isStateValid(const Position new_position);

  void setEnvironment();


  rrt_implement::ellipsoid robot_start;  //create robot

  std::vector<rrt_implement::ellipsoid> obstacles;
  rrt_implement::ellipsoid arena;
  rrt_implement::ellipsoid arena_collision;

  std::vector<rrt_implement::position> trajectory_;

  ros::ServiceClient client;

//publisher config
  ros::Publisher world_pub;
  ros::Publisher trajectory_pub;
}

#endif //PROJECT_PLANNER_H
