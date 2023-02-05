#include "path_planning.h"

namespace ns_path_planning {

PathPlanning::PathPlanning(ros::NodeHandle& handle) : _node_handle{handle} {}

CenterLine PathPlanning::run(const fsd_common_msgs::Map& map) {
    CenterLine center_line;
    { // Find Center Line
        center_line.polygon.points.clear();
        for (const auto& yellow: map.cone_yellow) {

            const auto it_blue = std::min_element(map.cone_blue.begin(), map.cone_blue.end(),
                                                  [&](const fsd_common_msgs::Cone &a,
                                                      const fsd_common_msgs::Cone &b) {
                                                      const double da = std::hypot(yellow.position.x - a.position.x,
                                                                                   yellow.position.y - a.position.y);
                                                      const double db = std::hypot(yellow.position.x - b.position.x,
                                                                                   yellow.position.y - b.position.y);

                                                      return da < db;
                                                  });

            geometry_msgs::Point32 p;
            p.x = static_cast<float>((yellow.position.x + it_blue->position.x) / 2.0);
            p.y = static_cast<float>((yellow.position.y + it_blue->position.y) / 2.0);
            p.z = 0.0;
            center_line.polygon.points.push_back(p);
        }
    }

    geometry_msgs::Polygon dense_center_line;
    { // Densify the center line
        const double      precision = 0.2;
        for (unsigned int i         = 1; i < center_line.polygon.points.size(); i++) {
            const double dx = center_line.polygon.points[i].x - center_line.polygon.points[i - 1].x;
            const double dy = center_line.polygon.points[i].y - center_line.polygon.points[i - 1].y;
            const double d  = std::hypot(dx, dy);

            const int         nm_add_points = d / precision;
            for (unsigned int j             = 0; j < nm_add_points; ++j) {
                geometry_msgs::Point32 new_p = center_line.polygon.points[i - 1];
                new_p.x += precision * j * dx / d;
                new_p.y += precision * j * dy / d;
                dense_center_line.points.push_back(new_p);
            }
        }
    }

    center_line.polygon = dense_center_line;
    center_line.header.frame_id = "map";
    center_line.header.stamp    = ros::Time::now();
    return center_line;
}
}