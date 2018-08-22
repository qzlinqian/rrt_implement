// to implement the rrt planner based on the configuration in rrt.h & rrt.cpp
// Created by lesley on 18-8-21.
//

#ifndef PROJECT_PLANNER_H
#define PROJECT_PLANNER_H

#include <rrt_implement/rrt.h>

namespace rrt{
  class rrtPlanner{
  private:
    RRTree RRT_Tree;
    rrtNode startPos;
    rrtNode goalPos;
  public:
    rrtPlanner(rrtNode &start, rrtNode &goal);
    rrtPlanner(){}

    rrtNode generateRandomNode();
    IDNumber findNearestNode(const rrtNode &inputNode);
    void stepAMetricForward();
    bool goalIsAchieved();
  };

  bool collisionChecking();
}

#endif //PROJECT_PLANNER_H
