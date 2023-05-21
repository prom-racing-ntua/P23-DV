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
    if (angle > 180.00)
    {
        return int(angle) - 360;
    }
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

    if (triangulation_object.is_infinite(starting_face))
    {

        Line_face_circulator lfc = triangulation_object.line_walk(position, direction_from_position);

        Face_handle first_face = --lfc;
        Face_handle second_face = --lfc;
        int index_1, index_2;
        for (int i = 0; i < 3; i++)
        {
            if (triangulation_object.is_infinite(first_face->vertex(i)))
            {
                index_1 = i;
            }
            if (triangulation_object.is_infinite(second_face->vertex(i)))
            {
                index_2 = i;
            }
        }
        Segment_2 seg_1(first_face->vertex((index_1 + 1) % 3)->point(), first_face->vertex((index_1 + 2) % 3)->point());
        Segment_2 seg_2(second_face->vertex((index_2 + 1) % 3)->point(), second_face->vertex((index_2 + 2) % 3)->point());
        float dist_1 = CGAL::squared_distance(seg_1, position);
        float dist_2 = CGAL::squared_distance(seg_2, position);
        my_edge starting_edge(Cone(Point(0, 0), -1), Cone(Point(0, 0), -1));
        if (dist_1 < dist_2)
        {
            starting_face = first_face->neighbor(index_1);
            Point other_a = first_face->vertex((index_1 + 1) % 3)->point();
            Point other_b = first_face->vertex((index_1 + 2) % 3)->point();
            starting_edge = my_edge(Cone(other_a, cone_lookup[other_a]),
                                    Cone(other_b, cone_lookup[other_b]));
        }
        else
        {
            starting_face = second_face->neighbor(index_2);
            Point other_a = second_face->vertex((index_2 + 1) % 3)->point();
            Point other_b = second_face->vertex((index_2 + 2) % 3)->point();
            starting_edge = my_edge(Cone(other_a, cone_lookup[other_a]),
                                    Cone(other_b, cone_lookup[other_b]));
        }

        not_visited_faces.erase(starting_face);
        Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
        best_best_path = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face, starting_face, 0, dir);
        best_best_path = filter_best_path(best_best_path, position, direction);
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
            other_b = starting_triangle.vertex((i + 2) % 3);
            my_edge starting_edge(Cone(other_a, cone_lookup[other_a]),
                                  Cone(other_b, cone_lookup[other_b]));

            if ((std::abs(angle_point_2(direction_from_position, position, starting_edge.midpoint())) < maximum_angle || CGAL::squared_distance(pos_vector, edges[i]) < 0.0001) && !triangulation_object.is_infinite(starting_face->neighbor(i)))
            {
                if(!triangulation_object.is_infinite(starting_face->neighbor(i)))
                {
                    Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
                    best_path[i] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(i), starting_face, 0, dir);
                    best_path[i] = filter_best_path(best_path[i], position, direction);
                }
                else{
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
            std::cout<<"all 3 invalid"<<std::endl;
            std::cout<<position<<" "<<direction_from_position<<" "<<starting_triangle.vertex(0)<<" "<<starting_triangle.vertex(1)<<" "<<starting_triangle.vertex(2)<<std::endl;
            return std::make_pair(empty_vector<Point>(),0);
            /*
            // std::cout << "all three edges invalid" << std::endl;
            Ray_2 pos_vector(position, direction_from_position);
            Segment_2 edges[3] = {Segment_2(starting_triangle.vertex(0), starting_triangle.vertex(1)),
                                  Segment_2(starting_triangle.vertex(1), starting_triangle.vertex(2)),
                                  Segment_2(starting_triangle.vertex(2), starting_triangle.vertex(0))};
            if (CGAL::squared_distance(pos_vector, edges[0]) < 0.00001)
            {
                my_edge starting_edge(Cone(starting_triangle.vertex(0), cone_lookup[starting_triangle.vertex(0)]),
                                      Cone(starting_triangle.vertex(1), cone_lookup[starting_triangle.vertex(1)]));
                Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
                best_path[2] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(2), starting_face, 0, dir);
                best_path[2] = filter_best_path(best_path[2], position, direction);
            }
            if (CGAL::squared_distance(pos_vector, edges[1]) < 0.00001)
            {
                my_edge starting_edge(Cone(starting_triangle.vertex(1), cone_lookup[starting_triangle.vertex(1)]),
                                      Cone(starting_triangle.vertex(2), cone_lookup[starting_triangle.vertex(2)]));
                Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
                best_path[0] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(0), starting_face, 0, dir);
                best_path[0] = filter_best_path(best_path[0], position, direction);
            }
            if (CGAL::squared_distance(pos_vector, edges[2]) < 0.00001)
            {
                my_edge starting_edge(Cone(starting_triangle.vertex(2), cone_lookup[starting_triangle.vertex(2)]),
                                      Cone(starting_triangle.vertex(0), cone_lookup[starting_triangle.vertex(0)]));
                Point dir(2 * starting_edge.midpoint().x() - position.x(), 2 * starting_edge.midpoint().y() - position.y());
                best_path[1] = find_best_path(position, direction, starting_edge, selected_edges, not_visited_faces, starting_face->neighbor(1), starting_face, 0, dir);
                best_path[1] = filter_best_path(best_path[1], position, direction);
            }
            */
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
    //cost_function_advanced(best_best_path.first, position, direction ,1);
    best_best_path = filter_best_path(best_best_path, position, direction);
    std::vector<Point> out;
    out.reserve(selected_edges.size() + 1);
    out.push_back(position);
    bool first=1;
    for (my_edge edge : best_best_path.first)
    {
        no_of_midpoints++;
        if(!first)out.push_back(edge.midpoint());
        else first=0;
    }
    last_calculated_path = out;
    
    return std::make_pair(out, best_best_path.second);
}

std::vector<Point> Triangulation::get_last_path() const
{
    return last_calculated_path;
}
int Triangulation::get_batch_number()const
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
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 1) % 3)) || current_face->neighbor((idx_opposite + 1) % 3) == starting_face)
    {
        // std::cout<<"1a";
        best_of_a.first.push_back(edge_1);
        best_of_a.second = cost_function(best_of_a.first, starting_point, starting_direction);
    }
    else if (/*CGAL::squared_distance(current_edge.midpoint(), edge_1.midpoint()) > maximum_distance * maximum_distance || std::abs(angle_point_2(current_direction, current_edge.midpoint(), edge_1.midpoint())) > maximum_angle ||*/ not_visited_faces.count(current_face->neighbor((idx_opposite + 1) % 3)) == 0 || current_depth >= target_depth)
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
    }
    if (triangulation_object.is_infinite(triangulation_object.mirror_vertex(current_face, (idx_opposite + 2) % 3)) || current_face->neighbor((idx_opposite + 2) % 3) == starting_face)
    {
        // std::cout<<"2a";
        //  if next face out of map OR next face == starting_face  add the edge, then stop
        best_of_b.first.push_back(edge_2);
        best_of_b.second = cost_function(best_of_b.first, starting_point, starting_direction);
    }
    else if (/*CGAL::squared_distance(current_edge.midpoint(), edge_2.midpoint()) > maximum_distance * maximum_distance || std::abs(angle_point_2(current_direction, current_edge.midpoint(), edge_2.midpoint())) > maximum_angle ||*/ not_visited_faces.count(current_face->neighbor((idx_opposite + 2) % 3)) == 0 || current_depth >= target_depth)
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
    }
    //best_of_a = filter_best_path(best_of_a, starting_point, starting_direction);
    //best_of_b = filter_best_path(best_of_b, starting_point, starting_direction);
    if (best_of_a.second < best_of_b.second)
    {
        return best_of_a;
    }
    return best_of_b;
}

std::pair<std::vector<my_edge>, int> Triangulation::filter_best_path(std::pair<std::vector<my_edge>, int> best_path, const Point &starting_position, const Direction_2 &starting_direction)
{
    if(best_path.second < filtering_threshold || best_path.first.size()==2) return best_path;
    best_path.first.pop_back();
    best_path.second = cost_function_advanced(best_path.first, starting_position, starting_direction);
    return filter_best_path(best_path, starting_position, starting_direction);
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
    //std::cout<<"maximum allowed angle is "<<maximum_angle<<std::endl;
    // if(total_number_of_edges == 21)std::cout << '[';
    for (int i = 0; i < total_number_of_edges; i++)
    {
        // if(total_number_of_edges == 21)std::cout << '(' << selected_edges[i].midpoint().x() << "," << selected_edges[i].midpoint().y() << "),";
        /* 1 */
        if (selected_edges[i].a().color == selected_edges[i].b().color)
        {
            // std::cout<<'*';
            cost += same_edge_penalty;
        }
        /* 2 */
        if (i == 0)
        {
            length = std::sqrt(CGAL::squared_distance(selected_edges[i].midpoint(), starting_position));
        }
        else
        {
            length = std::sqrt(CGAL::squared_distance(selected_edges[i].midpoint(), selected_edges[i - 1].midpoint()));
        }
        total_length += length;
        if (length < maximum_distance && length > minimum_distance)
            length_cost += length_penalty * length;
        else if (length > maximum_distance)
            cost += same_edge_penalty;
        /* 3 */
        /*
        if (i == 0)
        {
            //Point direction_from_position(starting_position.x() + starting_direction.dx(), starting_position.y() + starting_direction.dy());
            angle = std::abs(angle_point_2(starting_position, selected_edges[i].midpoint(), selected_edges[i+1].midpoint()));
            // if (angle < maximum_angle && angle>minimum_edge_angle)angle_cost += angle_penalty * angle;
            //max_angle = std::max(max_angle, angle);
            // else if (angle > maximum_angle) return INT_MAX - 1;
            angle_cost += angle_penalty * angle;
        }
        else if (i != total_number_of_edges - 1)
        {
            //Point direction_from_position(2 * selected_edges[i].midpoint().x() - selected_edges[i - 1].midpoint().x(), 2 * selected_edges[i].midpoint().y() - selected_edges[i - 1].midpoint().y());
            //angle = std::abs(angle_point_2(direction_from_position, selected_edges[i].midpoint(), selected_edges[i + 1].midpoint()));
            angle = std::abs(180 - angle_point_2(selected_edges[i-1].midpoint(),selected_edges[i].midpoint(),selected_edges[i+1].midpoint()));
            if(verbose)std::cout<<angle<<", ";
            max_angle = std::max(max_angle, angle);
            if (angle < maximum_edge_angle)
                angle_cost += angle_penalty * angle;
            else //if (angle > maximum_edge_angle)
            {
                if(verbose)std::cout<<std::endl<<"> *** <"<<std::endl;
                cost += same_edge_penalty;//if angle exceeds max angle, the penalty is applied to the total cost(to avoid scaling it by total_number_of_edges)
            }
        }*/
        if(i==1)angle = std::abs(180-angle_point_2(starting_position, selected_edges[i].midpoint(), selected_edges[i+1].midpoint()));
        else if (i!=total_number_of_edges-1 && i!=0)angle = std::abs(180 - angle_point_2(selected_edges[i-1].midpoint(),selected_edges[i].midpoint(),selected_edges[i+1].midpoint()));
        if(i!=total_number_of_edges-1 && i!=0)
        {
            if(verbose)std::cout<<angle<<", ";
            max_angle = std::max(max_angle, angle);
            if (angle < maximum_edge_angle)
                angle_cost += angle_penalty * angle;
            else //if (angle > maximum_edge_angle)
            {
                if(verbose)std::cout<<std::endl<<"> *** <"<<std::endl;
                cost += same_edge_penalty;//if angle exceeds max angle, the penalty is applied to the total cost(to avoid scaling it by total_number_of_edges)
            }
        }
    }
    if(verbose)
    {
        std::cout<<std::endl<<"Average angle: "<<(angle_cost / total_number_of_edges)/angle_penalty<<std::endl;
        std::cout<<"Max angle: "<<max_angle<<std::endl;
    }
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
    //std::cout<<"max angle: "<<max_angle<<std::endl;
    // if(std::abs(selected_edges[total_number_of_edges - 1].midpoint().y() + 1.26572) < 0.01 && total_number_of_edges == 21)std::cout<<"****^^^****"<<std::endl;
    return std::round(cost);
}


