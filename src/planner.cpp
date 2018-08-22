//
// Created by lesley on 18-8-21.
//

//#include <cstdio>
//#include <ctime>
//#include <cstdlib>
#include <random>
#include <rrt_implement/rrt.h>
#include <rrt_implement/planner.h>

rrt::rrtPlanner::rrtPlanner(rrt::rrtNode &start, rrt::rrtNode &goal):
        RRT_Tree(start.x, start.y, start.phi){
    startPos = start;
    goalPos = goal;
}

void rrt::rrtPlanner::generateRandomNode() {
    rrtNode randomNode;
    /*std::srand((unsigned)time(nullptr));
    randomNode.x = rand()/RAND_MAX * xRange; //get random x position
 * it's not safe, use c++11 random library instead */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, rrt::xRange);
}