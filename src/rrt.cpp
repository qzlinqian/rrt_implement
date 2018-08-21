//
// Created by Qian Lin on 18-8-21.
//

#include <rrt_implement/rrt.h>
#include <math.h>

rrt::RRTree::RRTree() {
    root = nullptr;
}

rrt::RRTree::RRTree(rrt::rrtNode *RootNode) {
    root = RootNode;
    root->father = nullptr;
}


