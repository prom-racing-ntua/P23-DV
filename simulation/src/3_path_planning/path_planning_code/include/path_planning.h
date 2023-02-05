#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include "geometry_msgs/PolygonStamped.h"
#include "fsd_common_msgs/Map.h"

namespace ns_path_planning {
using CenterLine = geometry_msgs::PolygonStamped;

class PathPlanning {
public:
    PathPlanning(ros::NodeHandle& handle);
    CenterLine run(const fsd_common_msgs::Map& map);

private:
    ros::NodeHandle _node_handle;
};
}

#endif //PATH_PLANNING_H