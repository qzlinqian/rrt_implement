#ifndef RRT_H
#define RRT_H

#include <vector>

namespace rrt{
    struct rrtNode{
        double x = 0;
        double y = 0;
        rrtNode* father = nullptr;
        std::vector<rrtNode*> children;
    };

    class RRTree{
    private:
        std::vector<rrtNode> rrtTree;
    public:
        rrtNode* root;

        // construction
        RRTree();
        RRTree(rrtNode* RootNode);

        //manipulation
        void insert(rrtNode &IntendedFather, rrtNode &IntendedChild);
        rrtNode* remove(rrtNode* ToBeRemoved);
        rrtNode* getNode(rrtNode* );
    };
}

#endif