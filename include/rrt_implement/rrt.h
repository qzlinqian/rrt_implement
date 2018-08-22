#ifndef RRT_H
#define RRT_H

#include <vector>

namespace rrt{

  using IDNumber = int;

  struct rrtNode{
    IDNumber NodeID;
    double x;
    double y;
    double phi;
    int father;
    std::vector<IDNumber> children;

    rrtNode(double xx=0, double yy=0, double ph=0, int ff=-1);
    rrtNode(const rrtNode &p);
  };

  class RRTree{
  private:
    static std::vector<rrtNode> rrtTree;
  public:
//        rrtNode* root;

    // construction
    RRTree(){};
    RRTree(double originX, double originY, double originPhi);

    //manipulation
    void insert(IDNumber IntendedFather, double xx, double yy, double ph); //know the father
    void insert(double xx, double yy, double ph);//not know the father
    rrtNode remove(IDNumber ToBeRemoved);
    std::vector<rrt::rrtNode>& getTree();
    rrtNode& getNode(IDNumber ID);
    int getTreeSize();
  };

  extern const double xRange,
      yRange,
      phiRange;

  extern const double MaxRange;

  extern const double xMetric,
      yMetric,
      phiMetric;

  double getEuclideanDistance(const rrtNode &Point1, const rrtNode &Point2);
}

#endif