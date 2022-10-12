#ifndef PATH_PLANNING_HANDLE_H
#define PATH_PLANNING_HANDLE_H

#include "path_planning.h"

namespace ns_path_planning
{
class PathPlanningHandle {
public:
    //Constructor
    explicit PathPlanningHandle(ros::NodeHandle& node_handle);

    //Getter
    int getNodeRate();

    //Public Methods    
    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();

private:
    ros::NodeHandle _node_handle;
    ros::Publisher _centerLinePublisher;
    ros::Subscriber _slamMapSubscriber;

    void slamMapCallback(const fsd_common_msgs::Map& map);

    std::string _slam_map_topic_name;
    std::string _center_line_topic_name;

    int _node_rate;
    PathPlanning _path_planning;
};
}

#endif //PATH_PLANNING_HANDLE_H