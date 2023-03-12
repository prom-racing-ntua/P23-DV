#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <utility>
#include <map>
#include <climits>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Kernel/global_functions.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Direction_2 Direction_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_t;
typedef Delaunay_t::Vertex_circulator Vertex_circulator;
typedef Delaunay_t::Vertex_handle Vertex_handle;
typedef Delaunay_t::Face_handle Face_handle;
typedef Delaunay_t::Face Face;
typedef Delaunay_t::Finite_vertex_handles Finite_vertex_handles;
typedef Delaunay_t::Finite_vertices_iterator Finite_vertices_iterator;
typedef Finite_vertex_handles::iterator Finite_vertex_handles_iterator;
typedef Delaunay_t::Point Point;
typedef Delaunay_t::Triangle Triangle;

struct Cone
{
    Point coords;
    int color;
    /*
        0 for yellow
        1 for blue
        2 for small orange
        3 for large orange
    */
    Cone() : coords(), color() {}
    Cone(Point x, int y) : coords(x), color(y) {}
    Cone(Point x) : coords(x), color(-1) {}
};

class my_edge
{
private:
    Cone a_priv;
    Cone b_priv;
    Point midpoint_priv;

public:
    my_edge(Cone x, Cone y)
    {
        a_priv = x;
        b_priv = y;
        midpoint_priv = CGAL::midpoint(a_priv.coords, b_priv.coords);
    }
    Cone a() const
    {
        return a_priv;
    }
    Cone b() const
    {
        return b_priv;
    }
    Point midpoint() const
    {
        return midpoint_priv;
    }
};

class Triangulation
{
private:
    std::map<Point, int> cone_lookup;
    Delaunay_t triangulation_object;
    std::vector<Point> last_calculated_path;
    int maximum_angle;
    int no_of_cones;
    int no_of_midpoints;
    int no_of_batches;
    // CGAL function for angle approximation supports only 3d points. Bridge function
    int angle_point_2(Point, Point, Point) const;

public:
    // creates either empty or full triangulation
    Triangulation(std::vector<Cone>);
    int cost_function(std::vector<my_edge> &, Point &, Direction_2 &) const;
    std::pair<std::vector<my_edge>, int> find_best_path(Point &, Direction_2 &, my_edge &, std::vector<my_edge>, std::unordered_set<Face_handle>, Face_handle, Face_handle &);
    std::vector<Point> new_batch(std::vector<Cone>, Point, Direction_2);
    std::vector<Point> get_last_path() const;
    // sends message to stdout/stderr/log for debug purposes
    ~Triangulation();
};

Triangulation::Triangulation(std::vector<Cone> initial_map)
{
    no_of_cones = 0;
    no_of_batches = 0;
    no_of_midpoints = 0;
    std::cout << "Object created ";
    maximum_angle = 45;

    if (!initial_map.empty())
    {
        std::cout << "with initial map." << std::endl;
        for (Cone cone : initial_map)
        {
            triangulation_object.insert(cone.coords);
        }
    }
    else
    {
        std::cout << "empty." << std::endl;
    }
}

Triangulation::~Triangulation()
{
    std::cout << "Object destroyed." << std::endl;
    std::cout << no_of_cones << "cones pushed." << std::endl
              << no_of_batches << "batches of cones." << std::endl
              << no_of_midpoints << "calculated." << std::endl;
}

int Triangulation::angle_point_2(Point a, Point b, Point c) const
{
    Kernel::Point_3 a_3(a.x(), a.y(), 0),
                    b_3(b.x(), b.y(), 0),
                    c_3(c.x(), c.y(), 0);

    double angle = CGAL::approximate_angle(a_3, b_3, c_3);
    if (angle > 180)
    {
        return int(angle) - 360;
    }
    else
    {
        return int(angle);
    }
}

std::vector<Point> Triangulation::new_batch(std::vector<Cone> local_map, Point position, Direction_2 direction)
{
    triangulation_object.clear(); // faster than "smarter" methods
    cone_lookup.clear();
    no_of_batches++;

    for (Cone cone : local_map)
    {
        no_of_cones++;
        triangulation_object.insert(cone.coords);
        cone_lookup[cone.coords] = cone.color;
    }
    // std::cout << "check_4" << std::endl;
    std::unordered_set<Face_handle> not_visited_faces;

    for (Delaunay_t::Finite_faces_iterator it = triangulation_object.finite_faces_begin();
         it != triangulation_object.finite_faces_end(); ++it)
    {
        not_visited_faces.insert(it);
    }
    // std::cout << "check_5" << std::endl;
    std::vector<my_edge> selected_edges;
    Face_handle starting_face = triangulation_object.locate(position);
    Triangle starting_triangle = triangulation_object.triangle(starting_face);
    Point other_a, other_b;
    Point direction_from_position(position.x() + direction.dx(), position.y() + direction.dy());
    // vector(position, direction_from_position) parallel to direction

    not_visited_faces.erase(starting_face);

    std::pair<std::vector<my_edge>, int> best_path[3];
    // std::cout << "check_6" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        other_a = starting_triangle.vertex((i + 1) % 3);
        other_b = starting_triangle.vertex((i + 2) % 3);
        my_edge starting_edge(Cone(other_a, cone_lookup[other_a]),
                              Cone(other_b, cone_lookup[other_b]));

        if (std::abs(angle_point_2(direction_from_position, position, starting_edge.midpoint())) > maximum_angle ||
            triangulation_object.is_infinite(starting_face->neighbor(i)))
        {
            std::vector<my_edge> empty_vector;
            best_path[i] = std::make_pair(empty_vector, INT_MAX);
            continue;
        }
        else
        {
            best_path[i] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(i), starting_face);
        }
    }
    int best_index;
    if (best_path[0].second < best_path[1].second && best_path[0].second < best_path[2].second)
    {
        best_index = 0;
    }
    else if (best_path[1].second < best_path[2].second)
    {
        best_index = 1;
    }
    else
    {
        best_index = 2;
    }
    std::vector<Point> out;
    out.reserve(selected_edges.size() + 1);
    out.push_back(position);
    for (my_edge edge : best_path[best_index].first)
    {
        no_of_midpoints++;
        out.push_back(edge.midpoint());
    }
    last_calculated_path = out;
    return out;
}

std::vector<Point> Triangulation::get_last_path() const
{
    return last_calculated_path;
}

std::pair<std::vector<my_edge>, int> Triangulation::find_best_path(Point &starting_point, Direction_2 &starting_direction,
                                                                   my_edge &current_edge, std::vector<my_edge> selected_edges, std::unordered_set<Face_handle> not_visited_faces, Face_handle current_face, Face_handle &starting_face)
{
    selected_edges.push_back(current_edge);
    not_visited_faces.erase(current_face);
    Point vertices[3] = {current_face->vertex(0)->point(), current_face->vertex(1)->point(), current_face->vertex(2)->point()};
    std::pair<std::vector<my_edge>, int> best_of_a = std::make_pair(selected_edges, INT_MAX),
                                         best_of_b = std::make_pair(selected_edges, INT_MAX);
    int idx_opposite;
    for (int i = 0; i < 3; i++)
    {
        if (vertices[i] != current_edge.a().coords && vertices[i] != current_edge.b().coords)
        {
            idx_opposite = i;
        }
    }
    // 1 refers to the opposite of the cw point of the opposite index. 2 to the ccw
    my_edge edge_1 = my_edge(Cone(vertices[idx_opposite], cone_lookup[vertices[idx_opposite]]),
                             Cone(vertices[(idx_opposite + 2) % 3], cone_lookup[vertices[(idx_opposite + 2) % 3]]));
    my_edge edge_2 = my_edge(Cone(vertices[idx_opposite], cone_lookup[vertices[idx_opposite]]),
                             Cone(vertices[(idx_opposite + 1) % 3], cone_lookup[vertices[(idx_opposite + 1) % 3]]));
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 1) % 3)) || edge_1.a().color == edge_1.b().color || current_face->neighbor((idx_opposite + 1) % 3) == starting_face)
    {
        best_of_a.first.push_back(edge_1);
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }
    else if (not_visited_faces.count(current_face->neighbor((idx_opposite + 1) % 3)) != 0)
    {
        best_of_a = find_best_path(starting_point, starting_direction, edge_1, best_of_a.first, not_visited_faces, current_face->neighbor((idx_opposite + 1) % 3), starting_face);
    }
    /*else{
        best_of_a.first.push_back(edge_1);
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }*/
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 2) % 3)) || edge_2.a().color == edge_2.b().color || current_face->neighbor((idx_opposite + 2) % 3) == starting_face)
    {
        best_of_b.first.push_back(edge_2);
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }
    else if (not_visited_faces.count(current_face->neighbor((idx_opposite + 2) % 3)) != 0)
    {
        best_of_b = find_best_path(starting_point, starting_direction, edge_2, best_of_b.first, not_visited_faces, current_face->neighbor((idx_opposite + 2) % 3), starting_face);
    }
    /*else{
        best_of_b.first.push_back(edge_2);
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }*/
    if (best_of_a.second < best_of_b.second)
    {
        return best_of_a;
    }
    return best_of_b;
}

int Triangulation::cost_function(std::vector<my_edge> &selected_edges, Point &starting_position, Direction_2 &starting_direction) const
{
    int cost = 0;
    // std::cout<<'[';
    for (my_edge edge : selected_edges)
    {
        // std::cout<<'('<<edge.midpoint.x()<<","<<edge.midpoint.y()<<"),";
        if (edge.a().color == edge.b().color)
        {
            // path crosses edge of same colored cones
            cost += 10;
        }
    }
    // std::cout<<']'<<std::endl<<cost<<std::endl;
    return cost;
}

int main()
{
    std::vector<Cone> initial;
    std::ifstream fs;
    int select;
    std::cin >> select;
    if (select == 0)
    {
        fs.open("input_simple.txt", std::ifstream::in);
    }
    else
    {
        fs.open("input.txt", std::ifstream::in);
    }
    Triangulation my_t(initial);
    std::vector<Cone> input;
    int t, c;
    double x, y;
    fs >> t;
    while (t--)
    {
        fs >> x >> y >> c;
        input.push_back(Cone(Point(x, y), c));
    }
    /*
    std::cout << '[';
    for (Cone cone : input)
    {
        if (cone.color == 0)
            std::cout << '(' << cone.coords.x() << ',' << cone.coords.y() << "),";
    }
    std::cout << ']' << std::endl
              << '[';
    for (Cone cone : input)
    {
        if (cone.color == 1)
            std::cout << '(' << cone.coords.x() << ',' << cone.coords.y() << "),";
    }
    std::cout << ']' << std::endl;
    */
    std::vector<Point> selected = my_t.new_batch(input, Point(0, 0), Direction_2(Delaunay_t::Segment(Point(0, 0), Point(0, 1))));
    for (Point point : selected)
    {
        std::cout << '(' << point.x() << "," << point.y() << ')' << ',';
    }
    std::cout << std::endl;
    fs.close();
    return 0;
}