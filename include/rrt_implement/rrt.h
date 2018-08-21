#ifndef RRT_H
#define RRT_H

#include <vector>

namespace rrt{
    struct rrtNode{
        double x;
        double y;
        rrtNode* father;
        std::vector<rrtNode*> children;

        rrtNode();
        rrtNode(double xx, double yy);
    };

    class RRTree{
    private:
        std::vector<rrtNode> rrtTree;
    public:
        rrtNode* root;

        // construction
        RRTree();
        RRTree(double originX, double originY);

        //manipulation
        void insert(rrtNode &IntendedFather, rrtNode &IntendedChild);
        int remove(rrtNode* ToBeRemoved);
        int getNode(rrtNode* );
    };
}

#endif