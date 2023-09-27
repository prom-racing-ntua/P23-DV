#include "sim_with_brake.hpp"
#include <random>
using namespace sim;

double State::calc_sa_r(double v_y, double r, double v_x) const
{
	if (v_x == 0)
		return 0;
	double lr = calc_lr(simplified, this->a_x);
	return std::atan((v_y - lr * r) / v_x);
}
double State::calc_sa_f(double v_y, double r, double v_x, double d) const
{
	if (v_x == 0)
		return 0;
	double lf = calc_lf(calc_lr(simplified, this->a_x));
	return std::atan((v_y + lf * r) / v_x) - d;
}
double sign(double x)
{
	return x == 0 ? 0 : x / std::abs(x);
}
double State::calc_fry(double sa_r, double frz) const
{
	const double P[] = {-1.9391836, 2.46810824, -0.21654031,
						5.53792739, 0.29777483, -0.87901614,
						-0.0990718, 7.02929697, -29.59686021,
						0.97520799, -8.20731378, 0.5275139,
						-1.0086604, -0.61898062, -55.82849645,
						70.38725604, 4.67966457, 0.9878318};
	const double ex = 0.001;
	const double Fz0 = 1112.0554070627252;
	double dfz = (frz - Fz0) / Fz0;
	const double IA = 0;
	double D = (abs(P[1]) + P[2] * dfz) * frz / (1 + P[3] * IA * IA);
	double C = abs(P[0]);
	double SVg = frz * (P[12] + P[13] * dfz) * IA;
	double Kg = frz * (P[16] + P[17] * dfz);
	double BCD = P[8] * Fz0 * std::sin(2.0 * std::atan(frz / ((P[9] + P[15] * IA * IA) * Fz0))) / (1.0 + P[10] * IA * IA);
	double Sh = (Kg * IA - SVg) / (BCD + ex);
	double ay = sa_r + Sh;
	double E = (P[4] + P[5] * dfz) * (1.0 + P[14] * IA * IA - (P[6] + P[7] * IA) * sign(ay));
	double B = BCD / (C * D + ex);
	double Sv = frz * (0 + 0 * dfz) + SVg;
	double psi = D * std::sin(C * std::atan(B * ay - E * (B * ay - std::atan(B * ay))));

	return psi + Sv;
}
double State::calc_ffy(double sa_r, double ffz) const
{
	const double P[] = {-1.9391836, 2.46810824, -0.21654031,
						5.53792739, 0.29777483, -0.87901614,
						-0.0990718, 7.02929697, -29.59686021,
						0.97520799, -8.20731378, 0.5275139,
						-1.0086604, -0.61898062, -55.82849645,
						70.38725604, 4.67966457, 0.9878318};
	const double ex = 0.001;
	const double Fz0 = 1112.0554070627252;
	double dfz = (ffz - Fz0) / Fz0;
	const double IA = 0;
	double D = (abs(P[1]) + P[2] * dfz) * ffz / (1 + P[3] * IA * IA);
	double C = abs(P[0]);
	double SVg = ffz * (P[12] + P[13] * dfz) * IA;
	double Kg = ffz * (P[16] + P[17] * dfz);
	double BCD = P[8] * Fz0 * std::sin(2.0 * std::atan(ffz / ((P[9] + P[15] * IA * IA) * Fz0))) / (1.0 + P[10] * IA * IA);
	double Sh = (Kg * IA - SVg) / (BCD + ex);
	double ay = sa_r + Sh;
	double E = (P[4] + P[5] * dfz) * (1.0 + P[14] * IA * IA - (P[6] + P[7] * IA) * sign(ay));
	double B = BCD / (C * D + ex);
	double Sv = ffz * (0 + 0 * dfz) + SVg;
	double psi = D * std::sin(C * std::atan(B * ay - E * (B * ay - std::atan(B * ay))));
	return psi + Sv;
}
double State::calc_ffz(double v_x) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);
	return constants.m * constants.g * lf / (lf + lr) + 0.25 * constants.P_air * constants.CdA * v_x * v_x;
}
double State::calc_frz(double v_x) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);
	return constants.m * constants.g * lr / (lf + lr) + 0.25 * constants.P_air * constants.CdA * v_x * v_x;
}
double State::calc_lr(bool simplified, double a_x) const
{
	a_x = simplified ? 0 : a_x / 9.81;
	double w = constants.m;
	double dwx = constants.h_cog * w * a_x / constants.wheelbase;
	double wf = w * constants.wd - dwx;

	// std::cout<<constants.wheelbase * constants.wd - constants.h_cog * a_x<<std::endl;
	return constants.wheelbase * (wf / w); // new wd = wf/w
}
double State::calc_lf(double l_r) const
{
	// std::cout<<constants.wheelbase - l_r<<std::endl;
	return constants.wheelbase - l_r;
}
double State::calc_drag(double v_x) const
{
	return 0.5 * constants.P_air * constants.CdA * v_x * v_x;
}
double State::calc_roll(double fz) const
{
	double coeff = 0.03;
	return coeff * fz;
}
double State::v_x_next(double v_x, double a_x, double dt) const
{
	return std::max(0.0, v_x + a_x * dt);
}
double State::v_y_next(double v_y, double a_y, double dt) const
{
	return v_y + a_y * dt;
}
double State::calc_a_x(double frx, double ffx, double f_drag, double f_roll, double ffy, double d, double v_y, double r) const
{
	// std::cout<<frx<<" "<<f_drag<<" "<<d<<" "<<std::sin(d)<<" "<<ffy * std::sin(d)<<" "<<v_y<<" "<<r<<std::endl;
	double ax_print = (frx + ffx - f_drag - f_roll - ffy * std::sin(d)) / constants.m + v_y * r;
	//std::cout << "frx, ffx and ax are: " << frx << " " << ffx << " " << ax_print << std::endl;
	return (frx + ffx - f_drag - f_roll - ffy * std::sin(d)) / constants.m + v_y * r;	
}
double State::calc_a_y(double fry, double ffx, double ffy, double d, double v_x, double r) const
{
	return (fry + ffy * std::cos(d)) / constants.m - v_x * r; //- ffx*std::sin(d)
}
double State::r_next(double ffy, double ffx, double d, double fry, double r, double dt) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);
	double r_dot = (ffy * lf * std::cos(d) - fry * lr) / constants.Iz; //+ffx*lf*std::sin(d)
	// std::cout<<r_dot<<std::endl;
	return r + r_dot * dt;
}
double State::theta_next(double r, double theta, double dt) const
{
	return theta + r * dt;
}
double State::x_next(double v_x, double theta, double v_y, double x, double dt) const
{
	double x_dot = v_x * std::cos(theta) - v_y * std::sin(theta);
	return x + x_dot * dt;
}
double State::y_next(double v_x, double theta, double v_y, double y, double dt) const
{
	double y_dot = v_x * std::sin(theta) + v_y * std::cos(theta);
	return y + y_dot * dt;
}
double State::s_next(double s, double v_x, double v_y, double a_x, double a_y, double dt) const
{
	double v = std::sqrt(v_x * v_x + v_y * v_y);
	double a = std::sqrt(a_x * a_x + a_y * a_y);
	return s + v * dt + 0.5 * a * dt * dt;
}

void State::next(double dt, double frx, double ffx, double delta)
{
	this->frx = frx;
	this->ffx = ffx;
	d = delta;
	sa_r = calc_sa_r(v_y, r, v_x);
	sa_f = calc_sa_f(v_y, r, v_x, delta);
	ffz = calc_ffz(v_x);
	frz = calc_frz(v_x);
	fry = calc_fry(sa_r, frz);
	ffy = calc_ffy(sa_f, ffz);
	double f_drag = calc_drag(v_x);
	double f_roll = calc_roll(frz + ffz);
	x = x_next(v_x, theta, v_y, x, dt);
	y = y_next(v_x, theta, v_y, y, dt);
	r = r_next(ffy, ffx, delta, fry, r, dt);
	theta = theta_next(r, theta, dt);
	s = s_next(s, v_x, v_y, a_x, a_y, dt);
	a_x = calc_a_x(frx, ffx, f_drag, f_roll, ffy, delta, v_y, r);
	a_y = calc_a_y(fry, ffx, ffy, delta, v_x, r);
	v_x = v_x_next(v_x, a_x, dt);
	v_y = v_y_next(v_y, a_y, dt);
	t += dt;
}

void State::check_ellipses(std::ostream &out) const
{
	// Full vehicle
	const double Fz = frz + ffz;
	const double Fz0 = 1112.0554070627252;
	const double dFz_full = (Fz - Fz0) / Fz0;
	const double mx_max_full = 0.66 * (2.21891927 - 1.36151651e-07 * dFz_full);
	const double my_max_full = 0.66 * (2.46810824 - 0.21654031 * dFz_full);
	const double fx = a_x * constants.m;
	const double fy = a_y * constants.m;
	out << (fx / mx_max_full) * (fx / mx_max_full) << " " << (fy / my_max_full) * (fy / my_max_full) << " " << Fz * Fz << std::endl;
	// Rear
	const double dFz = (frz - Fz0) / Fz0;
	const double mx_max = 0.66 * (2.21891927 - 1.36151651e-07 * dFz);
	const double my_max = 0.66 * (2.46810824 - 0.21654031 * dFz);
}

std::ostream &operator<<(std::ostream &out, const State &a)
{
	out << std::setw(7) << "t"
		<< /*"\t"*/ " " << std::setw(7) << "v_x"
		<< /*"\t"*/ " " << std::setw(7) << "v_y"
		<< /*"\t"*/ " " << std::setw(7) << "a_x"
		<< /*"\t"*/ " " << std::setw(7) << "a_y"
		<< /*"\t"*/ " " << std::setw(7) << "r"
		<< /*"\t"*/ " " << std::setw(7) << "sa_f"
		<< /*"\t"*/ " " << std::setw(7) << "sa_r"
		<< /*"\t"*/ " " << std::setw(7) << "ffy"
		<< /*"\t"*/ " " << std::setw(7) << "fry"
		<< /*"\t"*/ " " << std::setw(7) << "x"
		<< /*"\t"*/ " " << std::setw(7) << "y"
		<< /*"\t"*/ " " << std::setw(7) << "theta"
		<< /*"\t"*/ " " << std::setw(7) << "s"
		<< std::endl;
	out << std::setw(7) << a.t
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.v_x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.v_y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.a_x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.a_y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.r
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.sa_f
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.sa_r
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.ffy
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.fry
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.theta
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.s
		<< std::endl;
	return out;																																				
}


sim_node::sim_node() : Node("Simple_Simulation"), state(), constants(193.5, 250.0, 1.59, 0.66, 1.225, 2.0, 7.0, 3.9, 0.85, 0.2, 9.81, 0.275, 0.467, 4, 2, 0.079, 0.0735, 0.6, 0.025), global_idx(0), steering_dead_time(0.07), motor_dead_time(0.01), last_d(0), idx_of_last_lap(-1), sent(0), is_end(0)
{

    pub_pose = this->create_publisher<custom_msgs::msg::PoseMsg>("/pose", 10);
    timer_ = this->create_wall_timer(25ms, std::bind(&sim_node::timer_callback, this)); // 1kHz

    pub_map = this->create_publisher<custom_msgs::msg::LocalMapMsg>("/local_map", 10);

    pub_way = this->create_publisher<custom_msgs::msg::WaypointsMsg>("/waypoints", 10);

    pub_vel = this->create_publisher<custom_msgs::msg::VelEstimation>("/velocity_estimation", 10);

    pub_syst = this->create_publisher<custom_msgs::msg::TxSystemState>("/system_state", 10);

    pub_sens = this->create_publisher<custom_msgs::msg::RxVehicleSensors>("/sensor_data", 10);

    pub_steer = this->create_publisher<custom_msgs::msg::RxSteeringAngle>("/steering_angle", 10);

    pub_wheel = this->create_publisher<custom_msgs::msg::RxWheelSpeed>("/wheel_encoders", 10);

    pub_aut = this->create_publisher<custom_msgs::msg::AutonomousStatus>("/autonomous_status", 10);

    pub_miss = this->create_publisher<custom_msgs::msg::MissionSelection>("/mission_selection", 10);

    sub_comm = this->create_subscription<custom_msgs::msg::TxControlCommand>("/control_commands", 10, std::bind(&sim_node::command_callback, this, _1));

    state.init(constants, 0);

    std::ifstream fs;

    auto as = custom_msgs::msg::AutonomousStatus();
    as.id = 3;
    pub_aut->publish(as);

    declare_parameter<string>("discipline", "Autocross");
    string d = get_parameter("discipline").as_string();
    std::cout<<d<<std::endl;
    auto mission = custom_msgs::msg::MissionSelection();

    if (d == "Autocross")
    {
        discipline = 0;
        mission.mission_selected = 3;
    }
    else if (d == "Trackdrive")
    {
        discipline = 1;
        mission.mission_selected = 4;
    }
    else if (d == "Acceleration")
    {
        discipline = 2;
        mission.mission_selected = 1;
    }
    else if (d == "Skidpad")
    {
        discipline = 3;
        mission.mission_selected = 2;
    }
    else if (d == "EBS_Test")
    {
        discipline = 4;
        mission.mission_selected = 5;
    }
    else
    {
        discipline = 0;
        mission.mission_selected = 0;
    }

    pub_miss->publish(mission);

    /*
        Format:
            count
            x y color
            ...
    */
    if (discipline == 0 or discipline == 1)
    {
        fs.open("src/6.Controls/simple_sim/data/map.txt");
        state.x = -5.5;
    }
    else if (discipline == 2 or discipline == 4)
    {
        fs.open("src/6.Controls/simple_sim/data/Acceleration.txt");
        state.x = -1;
    }
    else if (discipline == 3)
    {
        fs.open("src/6.Controls/simple_sim/data/Skidpad.txt");
        state.x = -16;
    }

    std::cout<<"Discipline: "<<d<<" "<<discipline<<std::endl;
    int count;
    fs >> count;
    double x, y;
    int c;
    unseen_cones.reserve(count);
    for (int i = 0; i < count; i++)
    {
        fs >> x >> y >> c;
        if(discipline == 0)unseen_cones.push_back(Cone(x, y, c));
        else if(discipline == 4)
        {
            if(x<=20)seen_cones.push_back(Cone(x, y, c));
            else if(x>20 && x<=30)seen_cones.push_back(Cone(x, y, 2));
            else if(x>30 && x<=40)seen_cones.push_back(Cone(x, y, 3));
        }
        else seen_cones.push_back(Cone(x, y, c));
    }
    fs.close();

    for (Cone cone : unseen_cones)
    {
        if (cone.color == 0)
            std::cout << '(' << cone.x << "," << cone.y << "),";
    }
    std::cout << std::endl;
    for (Cone cone : unseen_cones)
    {
        if (cone.color == 1)
            std::cout << '(' << cone.x << "," << cone.y << "),";
    }
    std::cout << std::endl;

    log.open("src/6.Controls/simple_sim/data/log.txt");
    log << state;
    st_d_ticks = 2;
    mot_d_ticks = 2;

    std::ofstream log2;
    log2.open("src/6.Controls/simple_sim/data/log2.txt");
    // state.check_ellipses(log2);
    log2.close();
}

bool sim_node::lap_change() const
{
	double x = state.x;
	double y = state.y;
	if (discipline == 0 or discipline == 1)
	{
		return x >= 1 && x <= 3 && y >= -3 && y <= 3;
	}
	if (discipline == 2)
	{
		return x >= 75 && x <= 80;
	}
	if(discipline == 3)
	{
		return (x >= 0 && x <= 3 && y >= -3 && y <= 3) or ( x >= 10 && x <= 11 && y >= -3 && y <= 3);
	}
    return 0;
}

double add_noise(double x, double perc = 0.001)
{
	static std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, perc);
	double add = distribution(generator);
	// std::cout<<add<<std::endl;
	return x + add;
}

void sim_node::timer_callback()
{
    if(state.lap>=1 && state.v_x==0)
    {
        if(is_end==0)is_end = 1;
        if(is_end == 2)exit(0);
        auto autm = custom_msgs::msg::AutonomousStatus();
        autm.id = 4;
        pub_aut->publish(autm);
    }
	// std::cout << state.t << std::endl;
	/*
		1. state update 1kHz
		2. if 40Hz pose pub with noise
		3. if 2Hz map pub
	*/

	double frx_, ffx_, d, piston_area = 1e-3;
	double dt_=1e-3;
	for (int i = 0; i < 25; i++)
	{
		global_idx++;
		if (brake_press>0.0) {
			//std::cout << "asking to brake with brake_press -> " << brake_press << std::endl;
			//std::cout << "d_piston, R_disk_f, R_disk_r, R_wheel are -> " << " " << constants.d_piston << " " << constants.R_disk_f << " " <<  constants.R_disk_r << " " << constants.R_wheel << std::endl;
			//std::cout << "mi_disk, N_rear, N_front are -> " << " " << constants.mi_disk << " " << constants.N_rear << " " <<  constants.N_front << std::endl;
			double piston_area = 3.14159*std::pow(constants.d_piston/2,2);
			double pressure_input = brake_press*100000;
			double radius_ratio_f =  (constants.R_disk_f/constants.R_wheel);
			double radius_ratio_r =  (constants.R_disk_r/constants.R_wheel);
			frx_ = - pressure_input*piston_area*constants.mi_disk*constants.N_rear*radius_ratio_r;
			ffx_ = - pressure_input*piston_area*constants.mi_disk*constants.N_front*radius_ratio_f;
		}
		else {
			if (int(torques.size()) - mot_d_ticks - 1 < 0) frx_ = 0.0;
			else frx_ = torques[int(torques.size()) - mot_d_ticks - 1] * constants.gr * constants.eff / constants.R_wheel;
			ffx_ = 0.0;
		}
		if (int(steering.size()) - st_d_ticks - 1 < 0)
		{
			d = 0;
			// std::cout<<steering.size()<<" "<<st_d_ticks<<" /*/ ";
		}
		else
			d = steering[int(steering.size()) - st_d_ticks - 1];

		if (last_d + 0.0005 < d) last_d = last_d + 0.0005;
		else if (last_d - 0.0005 > d) last_d = last_d - 0.0005;
		last_d = std::min(3.14159 * 31.2 / 180, std::max(-3.14159 * 31.2 / 180, last_d));
		state.next(dt_, frx_, ffx_, last_d);
		if (lap_change() && global_idx - idx_of_last_lap > 2500)
		{
			std::cout << "----- Lap: " << ++state.lap << " -----" << std::endl;
			idx_of_last_lap = global_idx;
		}
	}
    auto msg = custom_msgs::msg::PoseMsg();
    auto pos = custom_msgs::msg::Point2Struct();
    auto vel = custom_msgs::msg::VelEstimation();
    auto msg2 = custom_msgs::msg::LocalMapMsg();
    auto s = custom_msgs::msg::ConeStruct();
    auto o = custom_msgs::msg::Point2Struct();
    auto sys = custom_msgs::msg::TxSystemState();
    auto dv = custom_msgs::msg::DriverlessStatus();
    auto sens = custom_msgs::msg::RxVehicleSensors();
    auto steer = custom_msgs::msg::RxSteeringAngle();
    auto wheel = custom_msgs::msg::RxWheelSpeed();

	// PUB POSE
    if (1 or global_idx % 25 == 0) //40Hz
    {
        /* LOGS */
        log << int(frx_) << "\t" << std::fixed << std::setprecision(3) << d << "\t" << std::fixed << std::setprecision(3) << last_d << std::endl;
        log << state;

        // std::ofstream log2;
        // log2.open("src/6.Controls/simple_sim/data/log2.txt", std::ios::app);
        // //state.check_ellipses(log2);
        // log2<<
        // log2.close();

        /* VEL_EST AND POSE*/
        pos.x = add_noise(state.x);
        pos.y = add_noise(state.y, 0.05);
        msg.position = pos;
        msg.theta = add_noise(state.theta, 0.01);
        vel.global_index = global_idx;

        vel.velocity_x = add_noise(state.v_x, 0.01);

        vel.velocity_y = add_noise(state.v_y, 0.01);
        vel.yaw_rate = add_noise(state.r, 1e-4);
        vel.acceleration_x = add_noise(state.a_x, 1e-4);
        vel.acceleration_y = add_noise(state.a_y, 1e-4);
        msg.velocity_state = vel;
        msg.lap_count = state.lap;
        pub_pose->publish(msg);
        pub_vel->publish(vel);

        /* SYSTEM */
        dv.id = 4;
        if(is_end==1)
        {
            dv.id = 6;
            is_end = 2;
        }
        sys.dv_status = dv;
        sys.vn_200_error = 0;
        sys.vn_300_error = 0;
        sys.camera_right_error = 0;
        sys.camera_left_error = 0;
        sys.clock_error = 0;
        sys.camera_inference_error = 0;
        sys.velocity_estimation_error = 0;
        sys.slam_error = 0;
        sys.mpc_controls_error = 0;
        sys.path_planning_error = 0;
        sys.pi_pp_controls_error = 0;
        sys.ins_mode = 2;
        sys.lap_counter = state.lap;
        sys.cones_count_actual = uint8_t(seen_cones.size());
        sys.cones_count_all = uint16_t(seen_cones.size());

        pub_syst->publish(sys);

        /* SENSOR DATA */
        sens.motor_torque_actual = int16_t(frx_ * constants.R_wheel / (constants.eff * constants.gr));
        sens.brake_pressure_front = brake_press;
        sens.brake_pressure_rear = brake_press;

        pub_sens->publish(sens);

        /* STEERING */
        steer.steering_angle = double(last_d) * 3.14159 / 180;

        pub_steer->publish(steer);

        /* WHEELS TBD*/

    }

	if (global_idx % 250 == 0)
	{
		std::cout << "Lap: "<< state.lap<< "\t\t" << state.t << "\t\t" << state.v_x << std::endl;
		// std::cout << '*' << std::endl;
		//  std::cout<<"> "<<unseen_cones.size()<<" "<<seen_cones.size()<<std::endl;
		for (int i = 0; i < unseen_cones.size(); i++)
		{
			double dsq = std::pow(state.x - unseen_cones[i].x, 2) + std::pow(state.y - unseen_cones[i].y, 2);
			if (dsq < 9 * 9 && dsq > 4 && std::acos((std::cos(state.theta) * (-state.x + unseen_cones[i].x) + std::sin(state.theta) * (-state.y + unseen_cones[i].y)) / std::sqrt(dsq)) < (3.14159 * 105 / 180))
			{
				seen_cones.push_back(unseen_cones[i]);
				unseen_cones.erase(unseen_cones.begin() + i);
				i--;
			}
		}

		msg2.cones_count_all = uint16_t(seen_cones.size());
		msg2.cones_count_actual = uint8_t(seen_cones.size());
		std::vector<custom_msgs::msg::ConeStruct> cs;
		cs.reserve(int(seen_cones.size()));

		for (auto i : seen_cones)
		{
			o.x = i.x;
			o.y = i.y;
			s.coords = o;
			s.color = i.color;
			cs.push_back(s);
		}
		msg2.local_map = cs;
		msg2.pose = msg;
		msg2.lap_count = state.lap;
		pub_map->publish(msg2);
	}
	/*
	if (sent <= 10 and state.lap==0)
	{
		std::ifstream mids;
		mids.open("src/6.Controls/simple_sim/data/accel_mids.txt");
		int cnt;
		mids >> cnt;
		std::vector<custom_msgs::msg::Point2Struct> wp;
		wp.reserve(cnt);
		custom_msgs::msg::Point2Struct sample;
		double x, y;
		for (int i = 0; i < cnt; i++)
		{
			mids >> x >> y;
			sample.x = x;
			sample.y = y;
			wp.push_back(sample);
		}
		auto msg5 = custom_msgs::msg::WaypointsMsg();
		msg5.count = cnt;
		msg5.initial_v_x = state.v_x;
		msg5.lap_count = state.lap;
		msg5.waypoints = wp;
		pub_way->publish(msg5);
		mids.close();
		sent++;
	}
	/*
	if (state.lap == 2 and sent == 11)
	{
		sent = 12;
		std::ifstream mids;
		mids.open("accel_mids.txt");
		int cnt;
		mids >> cnt;
		std::vector<custom_msgs::msg::Point2Struct> wp;
		// wp.reserve(cnt);
		custom_msgs::msg::Point2Struct sample;
		sample.x = state.x;
		sample.y = state.y;
		wp.push_back(sample);
		double x, y;
		for (int i = 0; i < cnt; i++)
		{
			mids >> x >> y;
			if (x > state.x)
			{
				sample.x = x;
				sample.y = y;
				wp.push_back(sample);
			}
		}
		auto msg5 = custom_msgs::msg::WaypointsMsg();
		msg5.count = cnt;
		msg5.initial_v_x = state.v_x;
		msg5.lap_count = state.lap;
		msg5.waypoints = wp;
		pub_way->publish(msg5);
	}
	*/
}
void sim_node::command_callback(const custom_msgs::msg::TxControlCommand::SharedPtr msg) {
	// std::cout << ">>> COMMAND <<<" << std::endl;
	torques.push_back(msg->motor_torque_target);
	steering.push_back(msg->steering_angle_target);
	brake_press = (float)msg->brake_pressure_target;
	std::ofstream log2;
	log2.open("src/6.Controls/simple_sim/data/log2.txt", std::ios::app);
	if(steering.size()>1)log2<<steering[steering.size()-1] - steering[steering.size()-2]<<std::endl;
	log2.close();
	// td::cout<<">>> "<<msg->steering_angle_target<<" "<<steering[steering.size()-1]<<std::endl;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sim_node>());
	rclcpp::shutdown();
	return 0;
}