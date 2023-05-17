#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
//#include <utility>
#include <map>
#include <climits>
#include <cmath>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/squared_distance_2.h>

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
typedef Delaunay_t::Finite_edges_iterator Finite_edges_iterator;
typedef Finite_vertex_handles::iterator Finite_vertex_handles_iterator;
typedef Delaunay_t::Point Point;
typedef Delaunay_t::Triangle Triangle;
typedef Delaunay_t::Line_face_circulator Line_face_circulator;

struct Cone {
    Point coords;
    int color;
    /*
        0 for yellow
        1 for blue
        2 for small orange
        3 for large orange
    */
    Cone(): coords(), color() {}
    Cone(Point x, int y): coords(x), color(y) {}
    Cone(Point x): coords(x), color(-1) {}
};

class my_edge {
private:
    Cone a_priv;
    Cone b_priv;
    Point midpoint_priv;

public:
    my_edge(Cone, Cone);

    Cone a() const {
        return a_priv;
    }
    Cone b() const {
        return b_priv;
    }
    Point midpoint() const {
        return midpoint_priv;
    }
};
//bridge function
int angle_point_2(const Point& a, const Point& b, const Point& c);

class Triangulation {
private:
    std::map<Point, int> cone_lookup; //matches points coords to their color
    Delaunay_t triangulation_object;
    std::vector<Point> last_calculated_path;
    int maximum_angle; //maximum angle for tree initialization
    int minimum_edge_angle; //DEPRECATED minimum angle to punish
    int minimum_distance;   //same as above, for distance
    int maximum_edge_angle; //above this angle the penalty increases
    int maximum_distance;   //same as above, for distance
    int no_of_cones;     //for stats
    int no_of_midpoints; // ^^
    int no_of_batches;   // ^^
    int target_depth; //max depth for the generated trees
    int same_edge_penalty;   //for the cost function
    float length_penalty;      // ^^
    float angle_penalty;       // ^^
    float total_length_reward;  // ^^
    int filtering_threshold; //above this threshold the selected path gets trimmed until it is below
    int cost_function(const std::vector<my_edge>&, const Point&, const Direction_2&) const;
    int cost_function_advanced(const std::vector<my_edge>&, const Point&, const Direction_2&, bool verbose = 0) const;
    std::pair<std::vector<my_edge>, int> find_best_path(const Point&, const Direction_2&, const my_edge&,  std::vector<my_edge>, std::unordered_set<Face_handle>, Face_handle, const Face_handle&, int, const Point &);
    std::pair<std::vector<my_edge>, int> filter_best_path(std::pair<std::vector<my_edge>, int>, const Point &, const Direction_2 &);

public:
    // creates either empty or full triangulation
    //Triangulation(int, int, int, int, int, int);
    Triangulation();
    void init(int maximum_angle = 90, int maximum_edge_angle = 45, int maximum_distance = 5, int minimum_angle = 0, int minimum_distance = 0, int target_depth = 10, int same_edge_penalty = 10, float length_penalty = 0.1, float angle_penalty = 0.1, float total_length_reward = 0.075, int filtering_threshold = 100);
    std::pair<std::vector<Point>, int> new_batch(const std::vector<Cone>&, const Point&, const Direction_2&);
    std::vector<Point> get_last_path() const;
    std::vector<Point> get_triangulation() const;
    int get_batch_number()const;
    // sends message to stdout/stderr/log for debug purposes
    ~Triangulation();
};