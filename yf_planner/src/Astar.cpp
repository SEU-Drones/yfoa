#include "Astar.h"

Astar::Astar()
{
}

Astar::~Astar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete AstarPathNodeMap_[i][j][k];
}

void Astar::init(std::string filename, InESDFMap::Ptr workspace)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["Astar"];
    int pool_size_x = (double)(yaml_node["pool_size_x"]);
    int pool_size_y = (double)(yaml_node["pool_size_y"]);
    int pool_size_z = (double)(yaml_node["pool_size_z"]);
    min_dist_ = (double)(yaml_node["min_dist"]);
    resolution_ = (double)(yaml_node["resolution"]);

    Eigen::Vector3i pool_size = Eigen::Vector3i{pool_size_x, pool_size_y, pool_size_z};
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
    AstarPathNodeMap_ = new AstarPathNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        AstarPathNodeMap_[i] = new AstarPathNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            AstarPathNodeMap_[i][j] = new AstarPathNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                AstarPathNodeMap_[i][j][k] = new AstarPathNode;
            }
        }
    }

    tie_breaker_ = 1.0 + 1.0 / 10000;
    workspace_ = workspace;
    inv_resolution_ = 1 / resolution_;

    rounds_ = 0;

    std::cout << "[Astar INIT] resolution: " << resolution_ << " (m)" << std::endl;
    std::cout << "[Astar INIT] min_dist: " << min_dist_ << " (m)" << std::endl;
    std::cout << "[Astar INIT] pool_size_x: " << pool_size_x << " (res)" << std::endl;
    std::cout << "[Astar INIT] pool_size_y: " << pool_size_y << " (res)" << std::endl;
    std::cout << "[Astar INIT] pool_size_z: " << pool_size_z << " (res)" << std::endl;
}

// void Astar::setPlanMap(PlanMapBase::Ptr workspace)
// {
// }

double Astar::getDiagHeu(AstarPathNodePtr node1, AstarPathNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = std::min(std::min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double Astar::getManhHeu(AstarPathNodePtr node1, AstarPathNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double Astar::getEuclHeu(AstarPathNodePtr node1, AstarPathNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

double Astar::getHeu(AstarPathNodePtr node1, AstarPathNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
    // return tie_breaker_ * getManhHeu(node1, node2);
}

std::vector<AstarPathNodePtr> Astar::retrievePath(AstarPathNodePtr current)
{
    std::vector<AstarPathNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

Eigen::Vector3d Astar::IndexToPos(Eigen::Vector3i index)
{
    return ((index - CENTER_IDX_).cast<double>() * resolution_) + center_;
};

bool Astar::PosToIndex(Eigen::Vector3d pt, Eigen::Vector3i &idx)
{
    idx = ((pt - center_) * inv_resolution_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        printf("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
        return false;
    }

    return true;
};

bool Astar::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!PosToIndex(start_pt, start_idx) || !PosToIndex(end_pt, end_idx))
        return false;

    if (isOccupied(IndexToPos(start_idx)))
    {
        std::cout << "[Astar] Start point is insdide an obstacle." << std::endl;
        do
        {
            start_pt = (start_pt - end_pt).normalized() * resolution_ + start_pt;
            if (!PosToIndex(start_pt, start_idx))
                return false;
        } while (isOccupied(IndexToPos(start_idx)));
    }

    if (isOccupied(IndexToPos(end_idx)))
    {
        std::cout << "[Astar] End point is insdide an obstacle." << std::endl;
        do
        {
            end_pt = (end_pt - start_pt).normalized() * resolution_ + end_pt;
            if (!PosToIndex(end_pt, end_idx))
                return false;
        } while (isOccupied(IndexToPos(end_idx)));
    }

    return true;
}

bool Astar::isOccupied(Eigen::Vector3d pos, double thr)
{
    if (thr < 0)
        return workspace_->isOccupied(pos);
    else
    {
        if (workspace_->getDist(pos) < thr)
            return true;
        else
            return false;
    }
}

bool Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ++rounds_;

    std::cout << "[astar]: start_pt = " << start_pt.transpose() << " end_pt = " << end_pt.transpose() << std::endl;

    // resolution_ = step_size;
    // inv_resolution_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Eigen::Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        printf("Unable to handle the initial or end point, force return!");
        return false;
    }

    AstarPathNodePtr startPtr = AstarPathNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    AstarPathNodePtr endPtr = AstarPathNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<AstarPathNodePtr, std::vector<AstarPathNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    AstarPathNodePtr neighborPtr = NULL;
    AstarPathNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = AstarPathNode::OPENSET; // put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); // put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    int num = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            gridPath_ = retrievePath(current);
            std::cout << "[astar]: search successful" << std::endl;
            std::cout << "[astar]: num_iter: " << num_iter << std::endl;
            std::cout << "[astar]: num: " << num << std::endl;
            return true;
        }
        current->state = AstarPathNode::CLOSEDSET; // move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    // int dz = 0;
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    num++;

                    Eigen::Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = AstarPathNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == AstarPathNode::CLOSEDSET)
                    {
                        continue; // in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (isOccupied(IndexToPos(neighborPtr->index), min_dist_))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    // double static_cost = std::abs(dx) + std::abs(dy) + std::abs(dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        // discover a new node
                        neighborPtr->state = AstarPathNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); // put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        // in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
    }

    std::cout << "[astar]: search failed" << std::endl;
    std::cout << "[astar]: num_iter: " << num_iter << std::endl;
    std::cout << "[astar]: num: " << num << std::endl;
    return false;
}

std::vector<Eigen::Vector3d> Astar::getPath()
{
    std::vector<Eigen::Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(IndexToPos(ptr->index));

    std::reverse(path.begin(), path.end());

    return path;
}

// 插值函数，根据指定的间隔生成新的路径点
std::vector<Eigen::Vector3d> Astar::getPath(double interval)
{
    std::vector<Eigen::Vector3d> originalPath = getPath();
    std::vector<Eigen::Vector3d> newPath;

    if (originalPath.empty())
    {
        return newPath;
    }

    // 添加第一个点
    newPath.push_back(originalPath[0]);

    for (size_t i = 1; i < originalPath.size(); ++i)
    {
        Eigen::Vector3d start = originalPath[i - 1];
        Eigen::Vector3d end = originalPath[i];
        double distance = (end - start).norm();

        // 计算需要插入的点数
        int numSegments = static_cast<int>(std::ceil(distance / interval));
        double step = distance / numSegments;

        for (int j = 1; j < numSegments; ++j)
        {
            Eigen::Vector3d newPoint = start + (end - start) * (j * step / distance);
            newPath.push_back(newPoint);
        }

        // 添加终点
        newPath.push_back(end);
    }

    return newPath;
}
