#include "sim_with_brake.hpp"
#include <random>

namespace sim{
bool has_hit_cone(double sin, double cos, double x, double y, double x_c, double y_c, double wb=1.59, double tr=1.22)
{
	const double x1 = x_c-x;
	const double y1 = y_c-y;
	const double xt = x1*cos+y1*sin;
	const double yt = x1*sin-y1*cos;
	// if((std::abs(xt)<(wb/2)&&std::abs(yt)<(tr/2)))
	// {
	// 	std::cout<<"x = "<<x<<", y = "<<y<<", x_c = "<<x_c<<", y_c = "<<y_c<<", phi = "<<std::arcsin(sin)<<std::endl;
	// 	return 1;
	// }
	// return 0;
	return std::abs(xt)<(wb/2)&&std::abs(yt)<(tr/2);
}


sim_node::sim_node() : Node("Simple_Simulation"), state(), constants(193.5, 250.0, 1.59, 0.66, 1.225, 2.0, 7.0, 3.9, 0.85, 0.2054, 9.81, 0.275, 0.467, 4, 2, 0.079, 0.0735, 0.6, 0.025,/*steering*/ 0.000279, 0.293, 0.0525, 11783, 0.052493, 0.015152, 0.06, 0.0, 27.98, 561.15, 50336, 973, 0.005, 0.0004, 0.00004, 100, 20, 30, 24*0.9, 5.78e-5, 0.01434, 0.0508, 0.002, 0.5, 0.8, 620), global_idx(0), steering_dead_time(0.07), motor_dead_time(0.01), last_d(0), idx_of_last_lap(-1), sent(0), is_end(0), total_doo(0), steering_model(), brake_model(), motor_model()
{
	parameter_load();
	pubs_and_subs();
	

	steering_model.init(steering_response, 
						get_parameter("steering_kp").as_double(), 
						get_parameter("steering_kd").as_double(), 
						constants, 
						get_parameter("steering_simple_model").as_bool(), 
						get_parameter("steering_model_integration_frequency").as_double());
	brake_model.init(brake_response, brake_velocity);
	motor_model.init(motor_response);

	state.init(constants, 0);

	std::ifstream fs;

    auto as = custom_msgs::msg::AutonomousStatus();
    as.id = 3;
    pub_aut->publish(as);

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
		state.x = -6.7;
	}
	else if (discipline == 2 or discipline == 4)
	{
		fs.open("src/6.Controls/simple_sim/data/Acceleration.txt");
		state.x = -2;
	}
	else if (discipline == 3)
	{
		fs.open("src/6.Controls/simple_sim/data/Skidpad.txt");
		state.x = -16.7;
	}

	std::cout<<"Discipline: "<<d<<" "<<discipline<<std::endl;
	int count;
	fs >> count;
		double x, y;
	std::cout<<count<<std::endl;
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

void sim_node::pubs_and_subs()
{
	pub_pose = this->create_publisher<custom_msgs::msg::PoseMsg>("/pose", 10);
	auto sim_period = std::chrono::duration<double, std::ratio<1, 1000> >(get_parameter("sim_node_period").as_double());
	timer_ = this->create_wall_timer(sim_period, std::bind(&sim_node::timer_callback, this)); // 1kHz

	pub_map = this->create_publisher<custom_msgs::msg::LocalMapMsg>("/local_map", 10);

	pub_way = this->create_publisher<custom_msgs::msg::WaypointsMsg>("/waypoints", 10);

	pub_vel = this->create_publisher<custom_msgs::msg::VelEstimation>("/velocity_estimation", 10);

	pub_syst = this->create_publisher<custom_msgs::msg::TxSystemState>("/system_state", 10);

	pub_sens = this->create_publisher<custom_msgs::msg::RxVehicleSensors>("/canbus/sensor_data", 10);

	pub_steer = this->create_publisher<custom_msgs::msg::RxSteeringAngle>("/canbus/steering_angle", 10);

	pub_wheel = this->create_publisher<custom_msgs::msg::RxWheelSpeed>("/canbus/wheel_encoders", 10);

	pub_aut = this->create_publisher<custom_msgs::msg::AutonomousStatus>("/canbus/autonomous_status", 10);

	pub_miss = this->create_publisher<custom_msgs::msg::MissionSelection>("/canbus/mission_selection", 10);

	sub_comm = this->create_subscription<custom_msgs::msg::TxControlCommand>("/control_commands", 10, std::bind(&sim_node::command_callback, this, _1));
}

void sim_node::parameter_load()
{
	declare_parameter<string>("discipline", "Autocross");
	
	declare_parameter<float>("perception_range", 10.0);
	perception_range = get_parameter("perception_range").as_double();

	declare_parameter<float>("brake_response", 10.0);
	brake_response = get_parameter("brake_response").as_double();

	declare_parameter<float>("brake_velocity", 10.0);
	brake_velocity = get_parameter("brake_velocity").as_double();

	declare_parameter<float>("steering_response", 10.0);
	steering_response = get_parameter("steering_response").as_double();

	declare_parameter<float>("steering_velocity", 10.0);
	steering_velocity = get_parameter("steering_velocity").as_double();

	declare_parameter<float>("motor_response", 10.0);
	motor_response = get_parameter("motor_response").as_double();

	declare_parameter<float>("steering_kp", 1000);
	declare_parameter<float>("steering_kd", 20);
	declare_parameter<bool>("steering_simple_model", 1);
	declare_parameter<float>("steering_model_integration_frequency", 10000.0);
	declare_parameter<float>("vehicle_model_integration_frequency", 1000.0);
	declare_parameter<float>("sim_node_period", 25.0);
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
		return (x >= 0 && x <= 3 && y >= -3 && y <= 3) /*or ( x >= 10 && x <= 11 && y >= -3 && y <= 3)*/;
	}
	if (discipline == 2)
	{
		return x >= 20 && x <= 25;
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
	int i=0;
	if(state.lap>=1 && state.v_x==0)
	{
		if(is_end==0)is_end = 1;
		if(is_end == 2)exit(0);
		auto autm = custom_msgs::msg::AutonomousStatus();
		autm.id = 4;
		pub_aut->publish(autm);
	}
	/*
		1. state update 1kHz
		2. if 40Hz pose pub with noise
		3. if 2Hz map pub
	*/

	double frx_, ffx_, d, piston_area = 1e-3;
	double dt_= 1 / get_parameter("vehicle_model_integration_frequency").as_double();
	for (int i = 0; i < 25 * 1e-3 * (1/dt_); i++)
	{
		global_idx++;

		/* Brake Pressure and Motor Torque*/
		
		brake_press = brake_model.get_command();
		if (brake_press>0.0) {
			double piston_area = 3.14159*std::pow(constants.d_piston/2,2);
			double pressure_input = brake_press*100000;
			double radius_ratio_f =  (constants.R_disk_f/constants.R_wheel);
			double radius_ratio_r =  (constants.R_disk_r/constants.R_wheel);
			frx_ = - pressure_input*piston_area*constants.mi_disk*constants.N_rear*radius_ratio_r;
			ffx_ = - pressure_input*piston_area*constants.mi_disk*constants.N_front*radius_ratio_f;
		}
		else {
			frx_ = motor_model.get_command()* constants.gr * constants.eff / constants.R_wheel;
			ffx_ = 0;
		}

		last_d = steering_model.get_command();
		last_d = std::min(3.14159 * 31.2 / 180, std::max(-3.14159 * 31.2 / 180, last_d));
		state.next(dt_, frx_, ffx_, last_d);
		
		steering_model.propagate(dt_, state.t*1e3, state.v_x, state.a_x);
		
		brake_model.propagate(dt_, state.t*1e3);
		motor_model.propagate(dt_, state.t*1e3);

		if (lap_change() && (global_idx - idx_of_last_lap)*state.v_x*1e-3 > 10)
		{
			std::cout << "----- Lap: " << ++state.lap << "\t-----" << std::endl;
			std::cout << "----- T = "  << state.t<<      "\t-----" << std::endl;
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
		log << int(frx_) << "\t" << std::fixed << std::setprecision(3) << steering_model.get_last_input() << "\t" << std::fixed << std::setprecision(3) << last_d << std::endl;
		log << state;

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

		/* Check if we are hitting cones*/
		double sin = std::sin(state.theta), cos = std::cos(state.theta);
		for(int i=0; i<seen_cones.size(); i++)
		{
			Cone cone = seen_cones[i];
			if(has_hit_cone(sin, cos, state.x, state.y, cone.x, cone.y))
			{
				if(discipline!=3)
				{
					seen_cones.erase(seen_cones.begin()+i);
					i--;
				}
				total_doo++;
				RCLCPP_WARN(get_logger(), "A cone has bin hit. Total count: %d", total_doo);
			}
		}
	}

	if (global_idx % 250 == 0)
	{
		double perc_r;
		std::cout << "Lap: "<< state.lap<< "\t\t" << state.t << "\t\t" << state.v_x << std::endl;
		for (int i = 0; i < unseen_cones.size(); i++)
		{
			double dsq = std::pow(state.x - unseen_cones[i].x, 2) + std::pow(state.y - unseen_cones[i].y, 2);
			// if(unseen_cones[i].color==0)
			// 	perc_r = 7;
			// else if(unseen_cones[i].color==1)
			// 	perc_r = 12;
			// else
				perc_r = perception_range;
			if (dsq < perc_r * perc_r && dsq > 4 && std::acos((std::cos(state.theta) * (-state.x + unseen_cones[i].x) + std::sin(state.theta) * (-state.y + unseen_cones[i].y)) / std::sqrt(dsq)) < (3.14159 * 105 / 180))
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

		for (int i=0; i<seen_cones.size(); i++)
		{
			o.x = seen_cones[i].x;
			o.y = seen_cones[i].y;
			s.coords = o;
			s.color = seen_cones[i].color;
			cs.push_back(s);
		}
		msg2.local_map = cs;
		msg2.pose = msg;
		msg2.lap_count = state.lap;
		pub_map->publish(msg2);
	}
}

void sim_node::command_callback(const custom_msgs::msg::TxControlCommand::SharedPtr msg) {
	torques.push_back(msg->motor_torque_target);
	motor_model.add_command(msg->motor_torque_target, state.t*1e3);
	// motor_model.add_command(20, state.t*1e3);

	steering.push_back(msg->steering_angle_target);
	steering_model.add_command(msg->steering_angle_target, state.t*1e3);
	// steering_model.add_command(0.4*std::sin(state.t), state.t*1e3);
	// steering_model.add_command((state.t<=2?0.1:-0.2), state.t*1e3);

	brake_model.add_command(msg->brake_pressure_target, state.t*1e3);
	// brake_model.add_command(0, state.t*1e3);
}
}
using namespace sim;
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sim_node>());
	rclcpp::shutdown();
	return 0;
}

