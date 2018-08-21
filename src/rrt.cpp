//
// Created by Qian Lin on 18-8-21.
//

#include <rrt_implement/rrt.h>
#include <math.h>

rrt::rrtNode::rrtNode() {
    x = 0;
    y = 0;
    father = nullptr;
}

rrt::rrtNode::rrtNode(double xx, double yy) {
    x = xx;
    y = yy;
    father = nullptr;
}



rrt::RRTree::RRTree() {
    root = nullptr;
}

rrt::RRTree::RRTree(double originX, double originY) {
    rrtNode temp(originX, originY);
    rrtTree.push_back(temp);
    root = &rrtTree.front();
    root->father = nullptr;
}


