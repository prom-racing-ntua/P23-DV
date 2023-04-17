#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
//#include <utility>
#include <string>
#include <map>
#include <climits>
#include <cmath>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/squared_distance_2.h>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"

#include "triangulation_cgal.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Direction_2 Direction_2;
typedef Kernel::Ray_2 Ray_2;
typedef Kernel::Segment_2 Segment_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_t;
typedef Delaunay_t::Vertex_handle Vertex_handle;
typedef Delaunay_t::Face_handle Face_handle;
typedef Delaunay_t::Face Face;
typedef Delaunay_t::Finite_vertex_handles Finite_vertex_handles;
typedef Delaunay_t::Finite_vertices_iterator Finite_vertices_iterator;
typedef Finite_vertex_handles::iterator Finite_vertex_handles_iterator;
typedef Delaunay_t::Point Point;
typedef Delaunay_t::Triangle Triangle;

using namespace std::chrono_literals;
using std::placeholders::_1;

class dummy_mapper: public rclcpp::Node
{
    public:
        dummy_mapper();
    private:
        rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr pub_map;
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_; 
        std::string file;
        int cone_count;
};

dummy_mapper::dummy_mapper():Node("Local_map")
{
    
}