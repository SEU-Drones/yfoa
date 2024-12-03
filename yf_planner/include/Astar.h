#ifndef ASTARSEARCH_H
#define ASTARSEARCH_H

#include <iostream>
#include <Eigen/Eigen>
#include <queue>
#include <opencv2/core/core.hpp>

// #include "Workspace.hpp"
#include "InESDFMap.hpp"

constexpr double inf = 1 >> 20;
struct AstarPathNode;
typedef AstarPathNode *AstarPathNodePtr;

struct AstarPathNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0}; // Distinguish every call
    enum enum_state state
    {
        UNDEFINED
    };
    Eigen::Vector3i index;

    double gScore{inf}, fScore{inf};
    AstarPathNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
    bool operator()(AstarPathNodePtr node1, AstarPathNodePtr node2)
    {
        return node1->fScore > node2->fScore;
    }
};

class Astar
{
public:
    Astar();
    ~Astar();

    // void init(WorkSpace::Ptr workspace, const Eigen::Vector3i pool_size, double dist_thr = -1.0);
    void init(std::string filename, InESDFMap::Ptr workspace);

    // void setPlanMap(PlanMapBase::Ptr workspace);
    void setMinDist(double mindist)
    {
        min_dist_ = mindist;
    }

    void setResolution(double resolution)
    {
        resolution_ = resolution;
    }

    bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getPath(double interval);

    typedef std::shared_ptr<Astar> Ptr;

private:
    std::vector<AstarPathNodePtr> gridPath_;

    InESDFMap::Ptr workspace_;

    int rounds_;
    double min_dist_;
    double resolution_, inv_resolution_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    double tie_breaker_;

    AstarPathNodePtr ***AstarPathNodeMap_;
    std::priority_queue<AstarPathNodePtr, std::vector<AstarPathNodePtr>, NodeComparator> openSet_;

    std::vector<AstarPathNodePtr> retrievePath(AstarPathNodePtr current);

    Eigen::Vector3d IndexToPos(Eigen::Vector3i index);
    bool PosToIndex(Eigen::Vector3d pt, Eigen::Vector3i &idx);

    bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    double getDiagHeu(AstarPathNodePtr node1, AstarPathNodePtr node2);
    double getManhHeu(AstarPathNodePtr node1, AstarPathNodePtr node2);
    double getEuclHeu(AstarPathNodePtr node1, AstarPathNodePtr node2);
    double getHeu(AstarPathNodePtr node1, AstarPathNodePtr node2);

    bool isOccupied(Eigen::Vector3d pos, double thr = -1.0);
};


#endif
