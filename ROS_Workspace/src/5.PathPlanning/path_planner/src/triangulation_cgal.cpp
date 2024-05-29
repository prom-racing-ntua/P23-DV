#include "triangulation_cgal.hpp"

my_edge::my_edge(Cone x, Cone y)
{
    a_priv = x;
    b_priv = y;
    midpoint_priv = CGAL::midpoint(a_priv.coords, b_priv.coords);
}

template <typename T>
std::vector<T> empty_vector()
{
    std::vector<T> empty;
    return empty;
}

Triangulation::Triangulation()
{
    no_of_cones = 0;
    no_of_batches = 0;
    no_of_midpoints = 0;
    std::cout << "Object created" << std::endl;
}

void Triangulation::init(int maximum_angle, int maximum_edge_angle, int maximum_distance, int minimum_angle, int minimum_distance, int target_depth, int same_edge_penalty, float length_penalty, float angle_penalty, float total_length_reward, int filtering_threshold)
{
    this->maximum_angle = maximum_angle;
    this->maximum_edge_angle = maximum_edge_angle;
    this->maximum_distance = maximum_distance;
    this->minimum_edge_angle = minimum_angle;
    this->minimum_distance = minimum_distance;
    this->target_depth = target_depth;
    this->same_edge_penalty = same_edge_penalty;
    this->length_penalty = length_penalty;
    this->angle_penalty = angle_penalty;
    this->total_length_reward = total_length_reward;
    this->filtering_threshold = filtering_threshold;
}

Triangulation::~Triangulation()
{
    std::cout << "Object destroyed." << std::endl;
    std::cout << no_of_cones << " cones pushed." << std::endl
              << no_of_batches << " batches of cones." << std::endl
              << no_of_midpoints << " midpoints calculated." << std::endl;
}

int angle_point_2(const Point &a, const Point &b, const Point &c)
{
    Kernel::Point_3 a_3(a.x(), a.y(), 0),
        b_3(b.x(), b.y(), 0),
        c_3(c.x(), c.y(), 0);

    double angle = CGAL::approximate_angle(a_3, b_3, c_3);
    // std::cout<<int(angle)<<std::endl;
    /*if (angle > 180.00)
    {
        return int(angle) - 360;
    }*/
    return int(angle);
}

std::pair<std::vector<Point>, int> Triangulation::new_batch(const std::vector<Cone> &local_map, const Point &position, const Direction_2 &direction)
{
    /*
    std::cout << "____________ (" << position.x() << "," << position.y() << ")" << std::endl
              << '[';

    for (Cone cone : local_map)
    {
        if (cone.color == 0)
            std::cout << '(' << cone.coords.x() << ',' << cone.coords.y() << "),";
    }
    std::cout << ']' << std::endl
              << '[';
    for (Cone cone : local_map)
    {
        if (cone.color == 1)
            std::cout << '(' << cone.coords.x() << ',' << cone.coords.y() << "),";
    }
    std::cout << ']' << std::endl;
    */
    // std::cout << local_map.size() << std::endl;
    triangulation_object.clear(); // faster than "smarter" methods
    cone_lookup.clear();
    no_of_batches++;

    for (Cone cone : local_map)
    {
        no_of_cones++;
        triangulation_object.insert(cone.coords);
        cone_lookup[cone.coords] = cone.color;
    }
    /*
    for(auto it= triangulation_object.finite_edges_begin();
                 it!=triangulation_object.finite_edges_end();
                 ++it)
        {
            Segment_2 edge = Segment_2(it->first->vertex((it->second+1)%3)->point(),it->first->vertex((it->second+2)%3)->point()) ;
            std::cout<<"("<<edge.vertex(0).x()<<","<<edge.vertex(0).y()<<"),("<<edge.vertex(1).x()<<","<<edge.vertex(1).y()<<")"<<std::endl;
        }
    */
    /*
  for (auto it = triangulation_object.finite_edges_begin(); it != triangulation_object.finite_edges_end(); it++)
  {
      std::cout << "polygon((" << triangulation_object.segment(it)[0].x() << "," << triangulation_object.segment(it)[0].y() << "),(" << triangulation_object.segment(it)[1].x() << "," << triangulation_object.segment(it)[1].y() << "))" << std::endl;
  }*/
    // std::cout << "check_4" << std::endl;
    std::unordered_set<Face_handle> not_visited_faces;

    for (Delaunay_t::Finite_faces_iterator it = triangulation_object.finite_faces_begin();
         it != triangulation_object.finite_faces_end(); ++it)
    {
        not_visited_faces.insert(it);
    }
    // std::cout << "check_5" << std::endl;
    std::vector<my_edge> selected_edges;

    Point direction_from_position(position.x() + direction.dx(), position.y() + direction.dy());

    Face_handle starting_face = triangulation_object.locate(position);

    std::pair<std::vector<my_edge>, int> best_best_path;

    out_of_convex = 0;

    if (triangulation_object.is_infinite(starting_face))
    {
        out_of_convex = 1;
        std::cout << "Exited convex hull" << std::endl;
        
        Face_handle infinite = starting_face;
        double min_distance = DBL_MAX, dist, cross;
        bool has_found_crossed = false;
        my_edge starting_edge(Cone(Point(0, 0), -1), Cone(Point(0, 0), -1));
        Ray_2 dir_vector(position, direction_from_position);
        Point a, b, mid, mid_p, mid_n, ab, perp;
        Segment_2 edge;
        for (auto it = triangulation_object.all_edges_begin();
             it != triangulation_object.all_edges_end();
             ++it)
        {
            
            a = it->first->vertex((it->second + 1) % 3)->point();
            b = it->first->vertex((it->second + 2) % 3)->point();
            edge = Segment_2(a, b);
            if(it->first->vertex((it->second + 1) % 3)==triangulation_object.infinite_vertex() || it->first->vertex((it->second + 2) % 3)==triangulation_object.infinite_vertex()){
                continue;
            }
            /*if((CGAL::squared_distance(a, Point(0,0))>1e8) || (CGAL::squared_distance(b, Point(0,0))>1e8)) //norm bigger than 10000m
            {
                std::cout<<"b"<<std::endl;
                continue;
            }*/
            else
            {
                dist = CGAL::squared_distance(position, edge);
                cross = CGAL::squared_distance(dir_vector, edge);
                if (has_found_crossed)
                {

                    if (cross < 1e-5)
                    {
                        if (dist < min_distance)
                        {
                            min_distance = dist;
                            starting_edge = my_edge(Cone(a, cone_lookup[a]),
                                                    Cone(b, cone_lookup[b]));
                        }
                    }
                }
                else
                {
                    if (cross < 1e-5 || dist < min_distance)
                    {
                        // has_found_crossed = true;
                        min_distance = dist;
                        starting_edge = my_edge(Cone(a, cone_lookup[a]),
                                                Cone(b, cone_lookup[b]));
                    }
                    has_found_crossed = cross < 1e-5;
                }
            }
        }
        if (DBL_MAX - min_distance < 1)
        {
            std::cout << "All distances infinite" << std::endl;
            return std::make_pair(empty_vector<Point>(), 0);
        }
        else
        {
            
            mid = starting_edge.midpoint();
            ab = Point(starting_edge.a().coords.x() - starting_edge.b().coords.x(), starting_edge.a().coords.y() - starting_edge.b().coords.y());
            if (ab.x() == 0)
                perp = Point(1, 0);
            else
                perp = Point(-ab.y() / ab.x(), 1);
            mid_p = Point(mid.x() + perp.x() / (100 * std::sqrt(perp.x()*perp.x()+perp.y()*perp.y())), mid.y() + perp.y() / (100 * std::sqrt(perp.x()*perp.x()+perp.y()*perp.y())));
            mid_n = Point(mid.x() - perp.x() / (100 * std::sqrt(perp.x()*perp.x()+perp.y()*perp.y())), mid.y() - perp.y() / (100 * std::sqrt(perp.x()*perp.x()+perp.y()*perp.y())));
            starting_face = triangulation_object.locate(mid_p);
            if (triangulation_object.is_infinite(starting_face))
            {
                starting_face = triangulation_object.locate(mid_n);
            }
            

            //not_visited_faces.erase(starting_face);
            //std::cout<<triangulation_object.triangle(starting_face)<<" "<<triangulation_object.triangle(infinite)<<" "<<starting_edge.a().coords<<" "<<starting_edge.b().coords<<std::endl;
            Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
            best_best_path = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face, infinite, 0, dir);
            //best_best_path = filter_best_path(best_best_path, position, direction);
        }
    }
    else
    {
        Triangle starting_triangle = triangulation_object.triangle(starting_face);
        Point other_a, other_b;

        // vector(position, direction_from_position) parallel to direction

        not_visited_faces.erase(starting_face);
        int count_invalid = 0; // for debugging
        std::pair<std::vector<my_edge>, int> best_path[3];
        Ray_2 pos_vector(position, direction_from_position);
        Segment_2 edges[3] = {Segment_2(starting_triangle.vertex(1), starting_triangle.vertex(2)),
                              Segment_2(starting_triangle.vertex(2), starting_triangle.vertex(0)),
                              Segment_2(starting_triangle.vertex(0), starting_triangle.vertex(1))};
        // std::cout << "check_6" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            other_a = starting_triangle.vertex((i + 1) % 3);
            //if(other_a.x()*other_a.x()+other_a.y()*other_a.y()<0.00001)std::cout<<other_a<<std::endl;
            other_b = starting_triangle.vertex((i + 2) % 3);
            //if(other_b.x()*other_b.x()+other_b.y()*other_b.y()<0.00001)std::cout<<other_b<<std::endl;
            
            my_edge starting_edge(Cone(other_a, cone_lookup[other_a]),
                                  Cone(other_b, cone_lookup[other_b]));

            if ((std::abs(angle_point_2(direction_from_position, position, starting_edge.midpoint())) < maximum_angle || CGAL::squared_distance(pos_vector, edges[i]) < 0.0001) && !triangulation_object.is_infinite(starting_face->neighbor(i)))
            {
                if (!triangulation_object.is_infinite(starting_face->neighbor(i)))
                {
                    Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
                    best_path[i] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(i), starting_face, 0, dir);
                    best_path[i] = filter_best_path(best_path[i], position, direction);
                }
                else
                {
                    std::vector<my_edge> vect{starting_edge};
                    int cost = cost_function_advanced(vect, position, direction);
                    best_path[i] = std::make_pair(vect, cost);
                }
            }
            else
            {
                // current_depth = already added edges
                count_invalid++;
                best_path[i] = std::make_pair(empty_vector<my_edge>(), INT_MAX);
                continue;
            }
        }
        if (count_invalid == 3)
        {
            std::cout << "all 3 invalid" << std::endl;
            std::cout << position << " " << direction_from_position << " " << starting_triangle.vertex(0) << " " << starting_triangle.vertex(1) << " " << starting_triangle.vertex(2) << std::endl;
            return std::make_pair(empty_vector<Point>(), 0);
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
        best_best_path = best_path[best_index];
    }
    // cost_function_advanced(best_best_path.first, position, direction ,1);
    if(!out_of_convex)best_best_path = filter_best_path(best_best_path, position, direction);
    std::vector<Point> out;
    out.reserve(selected_edges.size() + 1);
    out.push_back(position);
    bool first = 1;
    // for (my_edge edge : best_best_path.first)
    // {
    //     no_of_midpoints++;
    //     /*if(!first or selected_edges.size()==1)*/ out.push_back(edge.midpoint());
    // }
    int last_selected_index = 0;
    for(int i=0; i<best_best_path.first.size(); i++)
    {
        my_edge edge = best_best_path.first[i];
        
        if(i>0 && CGAL::squared_distance(edge.midpoint(), best_best_path.first[last_selected_index].midpoint()) < 2*2)
        {
            continue;
        }

        last_selected_index = i;
        no_of_midpoints++;
        out.push_back(edge.midpoint());
    }
    // if (out.size() >= 3)
    //     out.erase(out.begin());
    last_calculated_path = out;
    if (out.size() == 1)
    {
        std::cout << "Path has only one midpoint" << std::endl;
        // exit(1);
        return std::make_pair(empty_vector<Point>(), 0);
    }
    return std::make_pair(out, best_best_path.second);
}

std::vector<Point> Triangulation::get_last_path() const
{
    return last_calculated_path;
}
int Triangulation::get_batch_number() const
{
    return no_of_batches;
}

std::pair<std::vector<my_edge>, int> Triangulation::find_best_path(const Point &starting_point, const Direction_2 &starting_direction,
                                                                   const my_edge &current_edge, std::vector<my_edge> selected_edges, std::unordered_set<Face_handle> not_visited_faces,
                                                                   Face_handle current_face, const Face_handle &starting_face, int current_depth, const Point &current_direction)
{
    // std::cout << current_depth << std::endl;
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
    /*if(CGAL::squared_distance(edge_1.a().coords, edge_1.b().coords)>5.5*5.5*2)
    {
        std::cout<<"encountered big edge"<<std::endl<<"("<<current_edge.a().coords.x()<<","<<current_edge.a().coords.y()<<"),("<<current_edge.b().coords.x()<<","<<current_edge.b().coords.y()<<")"<<std::endl<<"("<<edge_1.a().coords.x()<<","<<edge_1.a().coords.y()<<"),("<<edge_1.b().coords.x()<<","<<edge_1.b().coords.y()<<")"<<std::endl;
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
        std::cout<<best_of_a.second<<std::endl;
    }
    else*/
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 1) % 3)) || current_face->neighbor((idx_opposite + 1) % 3) == starting_face)
    {
        // std::cout<<"1a";
        best_of_a.first.push_back(edge_1);
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }
    else if (CGAL::squared_distance(current_edge.midpoint(), edge_1.midpoint()) > maximum_distance * maximum_distance/* || std::abs(angle_point_2(current_direction, current_edge.midpoint(), edge_1.midpoint())) > maximum_angle*/ || not_visited_faces.count(current_face->neighbor((idx_opposite + 1) % 3)) == 0 || current_depth >= target_depth)
    {
        // if(CGAL::squared_distance(current_edge.midpoint(), edge_1.midpoint()) > maximum_distance * maximum_distance )std::cout<<"1b1 ";
        // if(std::abs(angle_point_2(current_direction, current_edge.midpoint(), edge_1.midpoint())) > maximum_angle )std::cout<<"1b2 ";
        // if(not_visited_faces.count(current_face->neighbor((idx_opposite + 1) % 3)) == 0 ) std::cout<<"1b3 ";
        // if(current_depth >= target_depth)std::cout<<"1b4 ";
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }
    else if (not_visited_faces.count(current_face->neighbor((idx_opposite + 1) % 3)) != 0)
    {
        Point dir(2 * edge_1.midpoint().x() - current_edge.midpoint().x(), 2 * edge_1.midpoint().y() - current_edge.midpoint().y());
        best_of_a = find_best_path(starting_point, starting_direction, edge_1, best_of_a.first, not_visited_faces, current_face->neighbor((idx_opposite + 1) % 3), starting_face, current_depth + 1, dir);
    }

    else
    {
        std::cout << "case not considered!" << std::endl;
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }
    /*if(CGAL::squared_distance(edge_2.a().coords, edge_2.b().coords)>5.5*5.5*2)
    {
        std::cout<<"encountered big edge"<<std::endl<<"("<<current_edge.a().coords.x()<<","<<current_edge.a().coords.y()<<"),("<<current_edge.b().coords.x()<<","<<current_edge.b().coords.y()<<")"<<std::endl<<"("<<edge_2.a().coords.x()<<","<<edge_2.a().coords.y()<<"),("<<edge_2.b().coords.x()<<","<<edge_2.b().coords.y()<<")"<<std::endl;
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
        std::cout<<best_of_b.second<<std::endl;
    }
    else */
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 2) % 3)) || current_face->neighbor((idx_opposite + 2) % 3) == starting_face)
    {
        // std::cout<<"2a";
        //  if next face out of map OR next face == starting_face  add the edge, then stop
        best_of_b.first.push_back(edge_2);
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }
    else if (CGAL::squared_distance(current_edge.midpoint(), edge_2.midpoint()) > maximum_distance * maximum_distance /*|| std::abs(angle_point_2(current_direction, current_edge.midpoint(), edge_2.midpoint())) > maximum_angle */ || not_visited_faces.count(current_face->neighbor((idx_opposite + 2) % 3)) == 0 || current_depth >= target_depth)
    {
        // std::cout<<"2b";
        //  if the next edge is above the max length, OR the formed angle is above the max angle OR reached target depth, dont add edge, stop
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }
    else if (not_visited_faces.count(current_face->neighbor((idx_opposite + 2) % 3)) != 0)
    {
        // BC=AB => (Xc-Xb, Yc-Yb) = (Xb - Xa, Yb - Ya) => 2Xb = Xc + Xa => Xc = 2Xb - Xa
        Point dir(2 * edge_2.midpoint().x() - current_edge.midpoint().x(), 2 * edge_2.midpoint().y() - current_edge.midpoint().y());

        best_of_b = find_best_path(starting_point, starting_direction, edge_2, best_of_b.first, not_visited_faces, current_face->neighbor((idx_opposite + 2) % 3), starting_face, current_depth + 1, dir);
    }

    else
    {
        std::cout << "case not considered!" << std::endl;
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }
    // best_of_a = filter_best_path(best_of_a, starting_point, starting_direction);
    // best_of_b = filter_best_path(best_of_b, starting_point, starting_direction);
    if (best_of_a.second < best_of_b.second)
    {
        return best_of_a;
    }
    return best_of_b;
}

std::pair<std::vector<my_edge>, int> Triangulation::filter_best_path(std::pair<std::vector<my_edge>, int> best_path, const Point &starting_position, const Direction_2 &starting_direction, int times_applied)
{
    if (best_path.second < filtering_threshold || best_path.first.size() == 2 /*|| times_applied>2*/)
        return best_path;
    best_path.first.pop_back();
    best_path.second = cost_function_advanced(best_path.first, starting_position, starting_direction);
    return filter_best_path(best_path, starting_position, starting_direction, times_applied + 1);
    
}

int Triangulation::cost_function(const std::vector<my_edge> &selected_edges, const Point &starting_position, const Direction_2 &starting_direction) const
{
    return cost_function_advanced(selected_edges, starting_position, starting_direction);
    int cost = 0;
    // std::cout << '[';
    for (my_edge edge : selected_edges)
    {
        // std::cout << '(' << edge.midpoint().x() << "," << edge.midpoint().y() << "),";
        if (edge.a().color == edge.b().color)
        {
            // path crosses edge of same colored cones
            cost += 10;
        }
    }
    // std::cout << ']' << std::endl
    // std::cout<< cost << std::endl;
    return cost;
}

int Triangulation::cost_function_advanced(const std::vector<my_edge> &selected_edges, const Point &starting_position, const Direction_2 &starting_direction, bool verbose) const
{
    float cost = 0; // negatives should add, positives subtract
    float color_cost = 0, length_cost = 0, angle_cost = 0;
    /*
        1. punish crossing edges of same color by same_edge_penalty for every invalid pair
        2. punish high length by length_penalty * midpoint_distance, then divide by total number of edges to obtain median length penalty (so as not to practically punish big paths)
        3. punish high angles by angle_penalty * angle_between_edges, then divide by total number of edges to obtain median angle penalty (so as not to practically punish big paths)
        4. reward big paths by subtracting size_reward * total size
    */
    int same_edge_penalty = this->same_edge_penalty; // placeholder
    float length_penalty = this->length_penalty;
    float total_length = 0;
    int total_number_of_edges = selected_edges.size();
    float angle_penalty = this->angle_penalty;
    float total_length_reward = this->total_length_reward;
    float length;
    float angle;
    float max_angle = 0;
    // std::cout<<"maximum allowed angle is "<<maximum_angle<<std::endl;
    //  if(total_number_of_edges == 21)std::cout << '[';
    for (int i = 0; i < total_number_of_edges; i++)
    {
        // if(total_number_of_edges == 21)std::cout << '(' << selected_edges[i].midpoint().x() << "," << selected_edges[i].midpoint().y() << "),";
        /* 1 */
        if ((selected_edges[i].a().color == selected_edges[i].b().color) && (selected_edges[i].a().color<=1))
        {
            // std::cout<<'*';
            cost += 2 * same_edge_penalty;
            if(verbose)std::cout<<"Same edge: "<<i<< '(' << selected_edges[i].a().coords.x() << "," << selected_edges[i].a().coords.y() << "),"<<'(' << selected_edges[i].b().coords.x() << "," << selected_edges[i].b().coords.y() << ")"<<std::endl;
        }
        /* 2 */

        if (i != 0)
        {
            length = std::sqrt(CGAL::squared_distance(selected_edges[i].midpoint(), selected_edges[i - 1].midpoint()));

            total_length += length;
            if (length < maximum_distance && length > minimum_distance)
                length_cost += length_penalty * length;
            else if (length > maximum_distance)
            {
                cost += same_edge_penalty;
                if(verbose) std::cout << "Max length: "<<i<<" , "<<length<<" m"<< '(' << selected_edges[i].a().coords.x() << "," << selected_edges[i].a().coords.y() << "),"<<'(' << selected_edges[i].b().coords.x() << "," << selected_edges[i].b().coords.y() << ")"<<std::endl;
            }
        }
        /* 3 */

        if (i == 1)
            angle = std::abs(180 - angle_point_2(starting_position, selected_edges[i].midpoint(), selected_edges[i + 1].midpoint()));
        else if (i != total_number_of_edges - 1 && i != 0)
            angle = std::abs(180 - angle_point_2(selected_edges[i - 1].midpoint(), selected_edges[i].midpoint(), selected_edges[i + 1].midpoint()));
        if (i != total_number_of_edges - 1 && i != 0)
        {
            
            max_angle = std::max(max_angle, angle);
            if (angle < maximum_edge_angle)
                angle_cost += angle_penalty * angle;
            else // if (angle > maximum_edge_angle)
            {
                if (verbose)
                    std::cout << "Max angle: "<<i<<" , "<<angle<<" deg"<< '(' << selected_edges[i].a().coords.x() << "," << selected_edges[i].a().coords.y() << "),"<<'(' << selected_edges[i].b().coords.x() << "," << selected_edges[i].b().coords.y() << ")"<<std::endl;
                cost += 10*same_edge_penalty; // if angle exceeds max angle, the penalty is applied to the total cost(to avoid scaling it by total_number_of_edges)
            }
        }
        /*correct orientation TBD*/
        bool f=0;
        if(i!=0)
        {
            if(CGAL::orientation(selected_edges[i-1].midpoint(), selected_edges[i].midpoint(), selected_edges[i].a().coords)==CGAL::RIGHT_TURN)
            {
                if(selected_edges[i].a().color==0)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            else if(CGAL::orientation(selected_edges[i-1].midpoint(), selected_edges[i].midpoint(), selected_edges[i].a().coords)==CGAL::LEFT_TURN)
            {
                if(selected_edges[i].a().color==1)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            if(CGAL::orientation(selected_edges[i-1].midpoint(), selected_edges[i].midpoint(), selected_edges[i].b().coords)==CGAL::RIGHT_TURN)
            {
                if(selected_edges[i].b().color==0)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            else if(CGAL::orientation(selected_edges[i-1].midpoint(), selected_edges[i].midpoint(), selected_edges[i].b().coords)==CGAL::LEFT_TURN)
            {
                if(selected_edges[i].b().color==1)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
        }
        else
        {
            if(CGAL::orientation(starting_position, selected_edges[i].midpoint(), selected_edges[i].a().coords)==CGAL::RIGHT_TURN)
            {
                if(selected_edges[i].a().color==0)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            else if(CGAL::orientation(starting_position, selected_edges[i].midpoint(), selected_edges[i].a().coords)==CGAL::LEFT_TURN)
            {
                if(selected_edges[i].a().color==1)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            if(CGAL::orientation(starting_position, selected_edges[i].midpoint(), selected_edges[i].b().coords)==CGAL::RIGHT_TURN)
            {
                if(selected_edges[i].b().color==0)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
            else if(CGAL::orientation(starting_position, selected_edges[i].midpoint(), selected_edges[i].b().coords)==CGAL::LEFT_TURN)
            {
                if(selected_edges[i].b().color==1)
                {
                    cost += 2*same_edge_penalty;    f=1;
                }
            }
        }
        if(f and verbose)std::cout<<"Wrong orientation "<<i<< '(' << selected_edges[i].a().coords.x() << "," << selected_edges[i].a().coords.y() << "),"<<'(' << selected_edges[i].b().coords.x() << "," << selected_edges[i].b().coords.y() << ") "<<selected_edges[i].a().color<<" "<<selected_edges[i].b().color<<std::endl;
    }
    /*if (verbose)
    {
        std::cout << std::endl
                  << "Average angle: " << (angle_cost / total_number_of_edges) / angle_penalty << std::endl;
        std::cout << "Max angle: " << max_angle << std::endl;
    }*/
    cost += length_cost / total_number_of_edges + angle_cost / total_number_of_edges - total_length * total_length_reward;
    // f(total_number_of_edges == 21)std::cout << ']' << std::endl;
    // if(total_number_of_edges == 21)std::cout<<cost<<std::endl;
    /*if(cost < 20)
    {
        std::cout<<std::endl;
        for(my_edge edge:selected_edges)
        {
            std::cout << '(' << edge.midpoint().x() << "," << edge.midpoint().y() << "),";
        }
        std::cout<<std::endl<<cost<<"\t color: "<<color_cost<<"\t length penalty: "<<length_cost / total_number_of_edges<<"\tangle: "<<angle_cost / total_number_of_edges<<"\t length reward: "<<total_length * total_length_reward<<std::endl;
    }*/
    // std::cout << std::endl << cost << "\t color: " << color_cost << "\t length penalty: " << length_cost / total_number_of_edges << "\tangle: " << angle_cost / total_number_of_edges << "\t length reward: " << total_length * total_length_reward <<" "<<total_length<< std::endl;
    // std::cout<<"max angle: "<<max_angle<<std::endl;
    // if(std::abs(selected_edges[total_number_of_edges - 1].midpoint().y() + 1.26572) < 0.01 && total_number_of_edges == 21)std::cout<<"****^^^****"<<std::endl;
    return std::round(cost);
}
