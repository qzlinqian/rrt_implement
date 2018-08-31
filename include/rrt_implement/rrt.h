#ifndef RRT_H
#define RRT_H

#include <vector>

namespace rrt{

  using IDNumber = int;

  struct Position{
    double x;
    double y;
    double phi;

    Position(double xx=0, double yy=0, double ph=0);
    Position(Position const &p);
  };

  struct rrtNode{
    IDNumber NodeID;
    double x;
    double y;
    double phi;
    int father;
//    std::vector<IDNumber> children;

    rrtNode(double xx=0, double yy=0, double ph=0, int ff=-1);
    rrtNode(rrtNode const &p);
  };

  class RRTree{
//  private:
  public:
    /*static*/ std::vector<rrtNode> rrtTree;
//  public:
//        rrtNode* root;

    // construction
    RRTree(){};
    RRTree(double originX, double originY, double originPhi);
    RRTree(Position const &originPos);

    //manipulation
    void insert(double xx, double yy, double ph, IDNumber IntendedFather=-1); //have father value passed: know the father
    void insert(const Position &pos, IDNumber IntendedFather=-1);
    rrtNode remove(IDNumber ToBeRemoved);
    rrtNode pop();
    std::vector<rrt::rrtNode>& getTree();
//    rrtNode& getNode(IDNumber ID);
    int getTreeSize();
//    rrtNode& getTopNode();
  };

  extern const double xRange,
      yRange,
      phiRange;

  extern const double MaxRange;

  extern const double xMetric,
      yMetric,
      phiMetric;

  double getEuclideanDistance(rrtNode const &Point1, Position const &Point2);
}

#endif