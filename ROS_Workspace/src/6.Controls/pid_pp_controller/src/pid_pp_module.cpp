#include <pid_pp_module.hpp>
using namespace pid_pp;

// Class Point
Point::Point(double a, double b)
    : x_priv(a), y_priv(b), error(0) {}

Point::Point()
    : x_priv(0), y_priv(0), error(0) {}

double Point::x() const
{
    return x_priv;
}

double Point::y() const
{
    return y_priv;
}

Point operator+(const Point &a, const Point &b)
{
    return Point(a.x() + b.x(), a.y() + b.y());
}
Point operator-(const Point &a, const Point &b)
{
    return Point(a.x() - b.x(), a.y() - b.y());
}
Point operator*(double a, const Point &b)
{
    return Point(a * b.x(), a * b.y());
}
Point Point::midpoint(const Point &a, const Point &b)
{
    return Point(0.5 * (a.x() + b.x()), 0.5 * (a.y() + b.y()));
}
double Point::distance(const Point &a, const Point &b)
{
    return std::pow(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2), 0.5);
}
std::ostream &pid_pp::operator<<(std::ostream &out, const Point &a)
{
    out<<"("<<a.x()<<","<<a.y()<<")";
    return out;
}

// Class PID
PID::PID()
    : request_sum(0), total_requests(0), error_integral(0), last_error(-1) {}

void PID::init(int kp, int ki, int kd, double dt, int damp, int integ_damp, int prop_damp, int der_damp)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    this->dt = dt;
    this->dampener = damp;
    proportional_dampener = prop_damp;
    integral_dampener = integ_damp;
    derivative_dampener = der_damp;
}

double PID::filter(double value, int dampener)
{
    if (value > dampener * 1.0)
        return dampener * 1.0;
    if (value < -dampener * 1.0)
        return -dampener * 1.0;
    return value;
}

double PID::operator()(double error)
{
    error_integral += error * dt;
    
    //std::cout<<"INTEGRAL = "<<(last_error)<<std::endl;
    double proportional = this->filter(error * Kp, proportional_dampener);
    double integral = this->filter(error_integral * Ki, integral_dampener);
    double derivative = (last_error == -1) ? 0 : (this->filter((error - last_error) / dt, derivative_dampener));

    double correction = this->filter(proportional + integral + derivative, dampener);
    request_sum += correction / 1000;
    total_requests++;
    last_error = error;
    //std::cout << "Error: " << error << ". Correction: " << correction << ". P: "<< proportional <<" "<<integral<<" "<<derivative<<" "<<proportional + derivative + integral<<std::endl;
    return correction;
}

double PID::operator()(double error, double dt)
{
    error_integral += error * dt;
    double proportional = this->filter(error * Kp, proportional_dampener);
    double integral = this->filter(error_integral * Ki, integral_dampener);
    double derivative = (last_error == -1) ? 0 : (this->filter((error - last_error) / dt, derivative_dampener));

    double correction = this->filter(proportional + integral + derivative, dampener);

    request_sum += correction / 1000;
    total_requests++;

    return correction;
}

void PID::flush_error()
{
    error_integral = 0;
}

PID::~PID()
{
    std::cout << "Average force request was: " << request_sum / total_requests << " kN." << std::endl;
}

// class PurePursuit
PurePursuit::PurePursuit()
    : request_sum(0), total_requests(0) {}

void PurePursuit::init(double ld_min, double ld_max, double v_min, double v_max, double wb, double emf)
{
    lookahead_min = ld_min;
    lookahead_max = ld_max;
    velocity_min = v_min;
    velocity_max = v_max;
    wheelbase = wb;
    emergency_factor = emf;
}

double PurePursuit::lookahead(double velocity, bool emergency) const
{
    double factor;
    if (emergency)
        factor = emergency_factor;
    else
        factor = 1;
    double ratio = (velocity - velocity_min) / (velocity_max - velocity_min);
    return std::min(lookahead_max, std::max(lookahead_min, factor * (lookahead_max * ratio + lookahead_min * (1 - ratio))));
}

double PurePursuit::operator()(const Point &target, double theta, double minimum_radius) const
{

    double ld = Point::distance(target, Point(0, 0));
    double heading;
    if(std::abs((-std::sin(theta) * target.x() + std::cos(theta) * target.y()))>0.001)
    {
        double turn_radius_1 = ld * ld / (2 * (-std::sin(theta) * target.x() + std::cos(theta) * target.y()));
        double turn_radius = turn_radius_1>0 ? std::max(turn_radius_1, minimum_radius) : std::min(turn_radius_1, -minimum_radius);

        heading = std::atan(wheelbase / turn_radius);
        //std::cout<<"target: ("<<target.x()<<", "<<target.y()<<"). R1: "<<turn_radius_1<<". Rmin:"<<minimum_radius<<" "<<ld<<" "<<theta<<std::endl;
    }else
    {
        heading = 0;
    }
    //std::cout<<"target: ("<<target.x()<<", "<<target.y()<<"). R1: "<<turn_radius_1<<". Rmin:"<<minimum_radius<<std::endl;
    return heading;
}

PurePursuit::~PurePursuit()
{
    std::cout << "Average steer angle request was: " << request_sum / total_requests << " rad." << std::endl;
}

// class Model
Model::Model(double mass, double grav, double wheelbase, double weight_distribution, double h_center_og, double air_density, double drag_coeff, double lift_coeff, double gear_ratio, double wheel_radius, double m_effectiveness, double Fz_0, double c_tire, double max_p_torque, double max_n_torque, double min_wdist)
    : m(mass), g(grav), wb(wheelbase), wd(weight_distribution), h_cog(h_center_og), p_air(air_density), cd_A(drag_coeff), cl_A(lift_coeff), gr(gear_ratio), R_wheel(wheel_radius), eff(m_effectiveness), Fz0(Fz_0), C_tire(c_tire), max_positive_torque(max_p_torque), max_negative_torque(max_n_torque), min_wd(min_wdist)
{
    max_positive_force = max_positive_torque * gr * eff / R_wheel;
    max_negative_force = max_negative_torque * gr * eff / R_wheel;
}

double Model::mx_max(double Fz) const
{
    const double dfz = (Fz - Fz0) / Fz0;
    const double mx_max = C_tire * (2.21891927 - 1.36151651e-07 * dfz);
    return mx_max;
}
double Model::my_max(double Fz) const
{
    const double dfz = (Fz - Fz0) / Fz0;
    const double my_max = C_tire * (2.46810824 - 0.21654031 * dfz);
    return my_max;
}

double Model::Fz_calc(std::string type, bool aero, bool moved_cog, double vx, double ax) const
{
    double weight = m * g;
    double aero_f = 0.5 * cd_A * vx * vx;
    double w_d = wd;
    ax = moved_cog?ax/9.81:0;

    double dwx = h_cog * weight * ax / wb; //weight transfer

    aero_f = aero?aero_f:0;

    if (type == "full")
    {
        return weight + aero_f;
    }
    else if (type == "front")
    {
        return weight*(1-wd) - dwx;
    }
    else if (type == "rear")
    {
        return weight*wd + dwx;
    }
    else
    {
        return -1;
    }
}

double Model::Torque(double F) const
{
    double t = (F * R_wheel) / (gr * eff);
    return std::min(max_positive_torque, std::max(max_negative_torque, t));
}

// class SplinePoints
SplinePoint::SplinePoint(Point pos, double s, double phi, double k)
    : position_priv(pos), s_priv(s), phi_priv(phi), k_priv(k) {}

SplinePoint::SplinePoint() {}

double SplinePoint::s() const
{
    return s_priv;
}

double SplinePoint::phi() const
{
    return phi_priv;
}

double SplinePoint::k() const
{
    return k_priv;
}

Point SplinePoint::position() const
{
    return position_priv;
}

double SplinePoint::target_speed() const
{
    return target_speed_priv;
}

void SplinePoint::set_target_speed(double v)
{
    target_speed_priv = v;
}

// class VelocityProfile TBD
VelocityProfile::VelocityProfile(path_planning::ArcLengthSpline &spline, double max_speed_, int samples_per_meter_, const Model &model_, double initial_speed, bool is_end, bool is_first_lap, double _safety_factor, double braking_distance)
    : model(&model_), max_speed(max_speed_), samples_per_meter(samples_per_meter_), last_visited_index(0), unknown(is_first_lap), safety_factor(_safety_factor)
{
    this->total_length = spline.getApproximateLength();
    // std::cout<<"TOTAL LENGTH = "<<total_length<<std::endl;
    int resolution = total_length * samples_per_meter;
    max_idx = resolution;
    path_planning::PointsData spline_data = spline.getSplineData(resolution = resolution);
    spline_samples = new SplinePoint[resolution];
    last_visited_index = 0;
    for (int i = 0; i < resolution; i++)
    {
        spline_samples[i] = SplinePoint(Point(spline_data(i, 0), spline_data(i, 1)), spline_data(i, 4), spline_data(i, 2), spline_data(i, 3));
        // std::cout<<spline_samples[i].position().x() <<" "<<spline_samples[i].position().y() <<" "<<spline_samples[i].s() <<" "<<spline_samples[i].phi() <<" "<<spline_samples[i].k() <<std::endl;
    }
    
    solve_profile(resolution, initial_speed, is_end, braking_distance);
    
    std::cout<<std::endl<<"------"<<std::endl;
    for (int i = 0; i < resolution; i++)
    {
        std::cout<<spline_samples[i].target_speed()<<", ";
    }
    std::cout<<std::endl<<"------"<<std::endl;
    //exit(0);
    
}

VelocityProfile::~VelocityProfile()
{
    delete[] spline_samples;
    // delete model;
}

std::pair<double, double> VelocityProfile::operator()(const Point &position, double theta)
{
    // return std::make_pair(0,0);
    int i = get_projection(position, theta);
    last_visited_index = i;
    i = std::min(i+1, max_idx); //looking at next target
    double cross_track = Point::distance(spline_samples[i].position(), position);
    double target = spline_samples[i].target_speed();
    return std::make_pair(target, cross_track);
}

int VelocityProfile::get_projection(const Point &position, double theta) const
{
    double min_error = DBL_MAX, min_error_index = -1, error_x, error_y, error;
    //std::cout << "max index = " << max_idx << std::endl;
    //for (int i = std::max(last_visited_index-2, 0); i < max_idx; i++)
    //for(int i = max_idx-1; i>= std::max(last_visited_index-2, 0); i--)
    //for(int i = max_idx-1; i>= 0; i--)
    for(int i = 0; i<max_idx; i++)
    {
        /*
        error_x = std::sin(spline_samples[i].phi())*(position.x() - spline_samples[i].position().x()) - std::cos(spline_samples[i].phi())*(position.y() - spline_samples[i].position().y());

        error_y = -std::cos(spline_samples[i].phi())*(position.x() - spline_samples[i].position().x()) - std::sin(spline_samples[i].phi())*(position.y() - spline_samples[i].position().y());
        */
        // std::cout<<spline_samples[i].position().x()<<" "<<position.x()<<" "<<spline_samples[i].position().y()<<" "<<position.y()<<std::endl;
        error_x = spline_samples[i].position().x() - position.x();
        error_y = spline_samples[i].position().y() - position.y();
        error = error_x * error_x + error_y * error_y;
        // std::cout<<error<<std::endl;
        if (error <= min_error)
        {
            min_error = error;
            min_error_index = i;
            if(error <= 0.05 * 0.05)break;
        }
    }
    if (min_error_index == -1)
        return 0;
    return std::min(int(min_error_index), max_idx-1);
}

Point VelocityProfile::get_target_point(double ld, const Point &position, double min_radius, double theta) const
{
    double closest_d = DBL_MAX;
    double dist, R;
    int sel_idx=0;
    double lr = model->wb * model->wd;
    Point closest_p, trans, rear = position - Point(-lr * std::cos(theta), -lr * std::sin(theta));
    for (int i = std::max(0, last_visited_index - 1); i < total_length * samples_per_meter; i++)
    //for(int i = total_length * samples_per_meter-1; i>=last_visited_index; i--)
    //for(int i = total_length * samples_per_meter-1; i>=0; i--)
    {
        dist = Point::distance(rear, spline_samples[i].position());
        trans = spline_samples[i].position() - rear;
        if(trans.x() * std::cos(theta) + trans.y() * std::sin(theta) < 0 )continue; //target point should be in front
        R = (ld * ld) / (2 * (-trans.x() * std::sin(theta) + trans.y() * std::cos(theta)));
        if (std::abs(ld - dist) <= closest_d && std::abs(R) > min_radius)
        {
            //if(closest_d<DBL_MAX && i - sel_idx > 100)continue; //Ds>10meters
            closest_d = std::abs(ld - dist);
            closest_p = trans;
            sel_idx = i;
            if(closest_d<=0.05)break;
        }
    }
    if (closest_d < DBL_MAX)
        return closest_p;
    
    //Retries for section before us
    for (int i = 0; i < std::max(0, last_visited_index - 1); i++)
    //for(int i = total_length * samples_per_meter-1; i>=last_visited_index; i--)
    //for(int i = total_length * samples_per_meter-1; i>=0; i--)
    {
        dist = Point::distance(rear, spline_samples[i].position());
        trans = spline_samples[i].position() - rear;
        R = (ld * ld) / (2 * (-trans.x() * std::sin(theta) + trans.y() * std::cos(theta)));
        if (std::abs(ld - dist) <= closest_d && std::abs(R) > min_radius)
        {
            //if(closest_d<DBL_MAX && i - sel_idx > 100)continue; //Ds>10meters
            closest_d = std::abs(ld - dist);
            closest_p = trans;
            sel_idx = i;
            if(closest_d<=0.05)break;
        }
    }
    if (closest_d < DBL_MAX)
        return closest_p;

    
    Point a(0, 0);
    std::cout<<"??"<<std::endl;
    a.error=1;
    return a;
    /* ELSE TBD */
}

Point VelocityProfile::get_last_projection()const
{
    return spline_samples[last_visited_index].position();
}

void VelocityProfile::solve_profile(int resolution, double initial_speed, bool is_end, double braking_distance)
{
    //std::cout<<'*';
    /* FIRST PASS */
    /* For formula derivation, check ADR-Controls intro*/
    // steady-state cornering asssumes no longit. acceleration, therefore my_max will be calculated with Fz = mg
    // mu^2k = my_max*mg => u=sqrt(my_max*g/k)
    double g = model->g;
    double m = model->m;
    double mu = model->my_max(m * g);
    mu = mu * safety_factor; //VP_SF1
    //log.open("k.txt", std::ios::app);
    for (int i = 0; i < resolution; i++)
    {
        if(is_end && spline_samples[i].s() * total_length>=braking_distance)
        {
            spline_samples[i].set_target_speed(0);
            continue;
        }
        double k = std::abs(spline_samples[i].k());
        if(k!=0)
        {
            spline_samples[i].set_target_speed(std::min(max_speed, std::sqrt(mu * g / k)));
            //log<<mu*g/k<<std::endl;
        }
        else
        {
            spline_samples[i].set_target_speed(max_speed);
            //log<<DBL_MAX<<std::endl;
        }
        if(spline_samples[i].s()*total_length>80)
        {
            //spline_samples[i].set_target_speed(0);
            //std::cout<<spline_samples[i].s()*total_length<<std::endl;
        }
    }
    //log.close();
    spline_samples[0].set_target_speed(std::max(1.0, initial_speed));
    if(unknown)spline_samples[resolution - 1].set_target_speed(0); // Safety Check
    /* SECOND PASS */
    /*
        The ellipse constraints will be calculated using single axis, so as to calculate the total a_x of the vehicle.
        The mx_pos_force, which is the Frx should be entered in the vx_dot formula to get max_engine_accel, before the minimum tire_accel and engine_accel is used.
        We calc the fz with a static cog because (a)no accel info is a priori available and (b) when accelerating the cog is moved back, which increases rear wheel friction. Thus, assuming a=0, the error drives us in safe territory.
        Downforce should be calculated with u=0, because if we use the local_speed, which is the upper limit, when the local_speed is decreased in passes 2 and 3, downforce will decrease and fz will decrease, decreasing the safe limit of a. By assuming the lower limit of the speed, aka 0, the error will increase fz, thus driving us in safe territory.
        Drag should be calculated with u=local_speed, because when the speed will decrease, drag will decrease, and the max_engine accel will increase. Thus by assuming the upper limit, the error will drive us in safe territory.
    */
    /*
        (fx/mx)**2 + (fy/my)**2 = fz**2
        (fx/mx)**2 = fz**2 - (fy/my)**2
        fx = mx * sqrt(fz**2 - (fy/my)**2)
    */
    /*
        ds = u_0*dt + 0.5*a*dt**2 => 0.5*a*dt**2+u_0*dt-ds=0
        dt = (-u_0+sqrt(u_0**2+2a*ds))/a
        u_1 = u_0 + a*dt = u_0 - u_0 + sqrt(u_0**2 + 2*a*ds)
    */
    double ds;
    double max_accel_eng, drag; //= model->max_positive_force / m;
    double local_accel, fy_req, fz_avail, local_speed, u_accel, fx_rem, fz_rear_avail, fx_rear_rem;
    for (int i = 0; i < resolution - 1; i++)
    {
        local_speed = spline_samples[i].target_speed();
        
        fy_req = m * std::pow(local_speed, 2) * std::abs(spline_samples[i].k());
        
        fz_avail = model->Fz_calc("full", 0, 0);
        fz_avail *= safety_factor; //VP_SF2
        
        drag = 0.5 * model->cd_A * std::pow(local_speed, 2);
        max_accel_eng = (model->max_positive_force - drag) / m;
        max_accel_eng *= safety_factor; //VP_SF3
        
        fx_rem = model->mx_max(fz_avail) * std::sqrt(std::max(0.0,std::pow(fz_avail, 2) - std::pow(fy_req / model->my_max(fz_avail), 2)));
        
        //std::cout<<fx_rem<<" "<<local_speed<<" "<<spline_samples[i+1].target_speed()<<std::endl;
        //if(std::isnan(fx_rem)&&!std::isnan(local_speed))exit(50);
        fz_rear_avail = model->Fz_calc("rear", 0, 0);
        fz_rear_avail *= safety_factor; //VP_SF4
        fx_rear_rem = model->mx_max(fz_rear_avail) * fz_rear_avail;//available fx assuming 0 fy
        fx_rem = std::min(fx_rem, fx_rear_rem);

        local_accel = std::min(fx_rem / m, max_accel_eng);
        
        ds = (spline_samples[i + 1].s() - spline_samples[i].s()) * total_length;
        u_accel = std::sqrt(local_speed * local_speed + 2 * local_accel * ds);

        spline_samples[i + 1].set_target_speed(std::min(u_accel, spline_samples[i + 1].target_speed()));
        
    }
    /* THIRD PASS */
    /*
        The ellipse constraints will be calculated using single axis, so as to calculate the total -a_x of the vehicle.
        The mx_neg_force, which is the Frx should be entered in the vx_dot formula to get max_engine_decel, before the minimum tire_decel and engine_decel is used.
        We calc the fz with the minimum weight distribution. No decel is known beforehand so by assuming the largest probable decel, which pushes the cog the most forward, leaving the least available fz in the rear wheels. therefore the braking applied will definitely not lock up tires, and the error will drive us in safe territory.
        Drag and downforce should be calculated with u=local speed (the one obtained by passes 1 and 2) because at each step, the effect of the 3rd pass has already been calculated so local_speed = final speed.
    */
    double max_decel_eng = model->max_negative_force / m;
    double local_decel, u_decel;
    for (int i = resolution - 1; i > 0; i--)
    {
        local_speed = spline_samples[i].target_speed();

        fy_req = m * std::pow(local_speed, 2) * std::abs(spline_samples[i].k());

        fz_avail = model->Fz_calc("full", 1, 1, local_speed, model->min_wd);
        fz_avail *= safety_factor; //VP_SF5

        fx_rem = - model->mx_max(fz_avail) * std::sqrt(std::max(0.0,std::pow(fz_avail, 2) - std::pow(fy_req / model->my_max(fz_avail), 2)));

        drag = 0.5 * model->cd_A * std::pow(local_speed, 2);
        max_decel_eng = (model->max_negative_force + drag) / m;
        max_decel_eng *= safety_factor; //VP_SF6

        fz_rear_avail = model->Fz_calc("rear", 1, 1, local_speed);
        fz_rear_avail *= safety_factor; //VP_SF7
        fx_rear_rem = -model->mx_max(fz_rear_avail)*fz_rear_avail;
        fx_rem = std::max(fx_rem, fx_rear_rem);

        local_decel = std::max(fx_rem / m, max_decel_eng);

        //std::cout<<local_decel<<std::endl;

        ds = (spline_samples[i].s() - spline_samples[i - 1].s())*total_length;

        //std::cout<<local_speed - u_decel<<std::endl;
        u_decel = std::sqrt(local_speed * local_speed - 2 * local_decel * ds);
        
        spline_samples[i - 1].set_target_speed(std::min(u_decel, spline_samples[i - 1].target_speed()));
    }

    
}