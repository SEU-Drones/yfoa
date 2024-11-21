#include "PlanFSM.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planfsm");
    ros::NodeHandle node("~");

    std::string filename;
    node.param<std::string>("paramfile/path", filename, "./src/yf_manager/config/yf.yaml");

    PlanFSM planner;
    planner.init(filename, node);

    ros::spin();
    return 0;
}