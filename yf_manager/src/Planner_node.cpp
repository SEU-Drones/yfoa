#include "Planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner");
    ros::NodeHandle node("~");

    std::string filename;
    node.param<std::string>("paramfile/path", filename, "./src/yf_manager/config/yf.yaml");

    Planner planner;
    planner.init(filename, node);

    ros::spin();
    return 0;
}