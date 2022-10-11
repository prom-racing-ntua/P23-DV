#include "path_planning_handler.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "purePursuit");
    ros::NodeHandle nodeHandle("~");
    ns_path_planning::PathPlanningHandle pathPlanningHandle(nodeHandle);
    ros::Rate loop_rate(pathPlanningHandle.getNodeRate());
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}