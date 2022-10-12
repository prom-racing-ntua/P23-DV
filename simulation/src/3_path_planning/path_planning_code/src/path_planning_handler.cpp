#include "path_planning_handler.h"

namespace ns_path_planning {

PathPlanningHandle::PathPlanningHandle(ros::NodeHandle& node_handle) : _node_handle{node_handle}, _path_planning{node_handle} {
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

void PathPlanningHandle::loadParameters() {
    if (!_node_handle.param<std::string>("slam_map_topic_name",
                                        _slam_map_topic_name,
                                        "/estimation/slam/map")) {
        ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << _slam_map_topic_name);
    }
    if (!_node_handle.param<std::string>("center_line_topic_name",
                                        _center_line_topic_name,
                                        "/control/pure_pursuit/center_line")) {
        ROS_WARN_STREAM("Did not load center_line_topic_name. Standard value is: " << _center_line_topic_name);
    }
    if (!_node_handle.param("node_rate", _node_rate, 1)) {
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << _node_rate);
    }
}

void PathPlanningHandle::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    _slamMapSubscriber = _node_handle.subscribe(_slam_map_topic_name, 1, &PathPlanningHandle::slamMapCallback, this);
}

void PathPlanningHandle::publishToTopics() {
    _centerLinePublisher = _node_handle.advertise<geometry_msgs::PolygonStamped>(_center_line_topic_name, 1, true);
}

void PathPlanningHandle::slamMapCallback(const fsd_common_msgs::Map& map) {
    CenterLine center_line = _path_planning.run(map);
    _centerLinePublisher.publish(center_line);
}

int PathPlanningHandle::getNodeRate() {return _node_rate;}
}