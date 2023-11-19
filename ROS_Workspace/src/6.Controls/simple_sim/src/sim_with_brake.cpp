#include "sim_with_brake.hpp"
#include <random>

namespace sim{
/* STATE */

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
	return constants.m * constants.g * lf / (lf + lr) + 0.25 * constants.P_air * constants.ClA * v_x * v_x;
}
double State::calc_frz(double v_x) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);
	return constants.m * constants.g * lr / (lf + lr) + 0.25 * constants.P_air * constants.ClA * v_x * v_x;
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

std::pair<double, double> State::get_mx_my(double vx, double ax)
{
	// Full vehicle
	const double Fz = 193.5 * 9.81 * 0.467 - 0.28 * 193.5 * ax / 1.59  + 0.5 * 1.225 * 2 * vx * vx;
	const double Fz0 = 1112.0554070627252;
	const double dFz = (Fz - Fz0) / Fz0;
	const double mx_max = 0.66 * (2.21891927 - 1.36151651e-07 * dFz);
	const double my_max = 0.66 * (2.46810824 - 0.21654031 * dFz);
	return std::make_pair(mx_max, my_max);
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



template<typename T>
Actuation<T>::Actuation(): delay_time(0), speed(0), value(0) {}
template<typename T>
void Actuation<T>::init(double delay, double approach_speed)
{
	delay_time = delay;
	speed = approach_speed;
}
void motorActuation::init(double delay) {this->Actuation<double>::init(delay, 0.0);}
template<typename T>
void Delay<T>::init(double delay) {this->Actuation<T>::init(delay, 0.0);}
template<typename T>
void Actuation<T>::add_command(T command, double time)
{
	commands_log.push_back(std::make_pair(command, time));
}
template<typename T>
T Actuation<T>::get_command()const
{
	if (std::isnan(value))
	{
		std::cout<<"The actuators fucked up"<<std::endl;
		exit(1);
	}
	return value;
}
void steeringActuation::init(double delay, double kp, double kd, Constants a, bool simplified)
{
	this->steering_model.init(a, kp, 0, kd, 0.001, 3000);
	delay_time = delay;
	this->simplified = simplified;
}


void steeringActuation::propagate(double dt, double time, double vx, double ax)
{
	if(commands_log.size() == 0)return ;
	if(commands_log[0].second + delay_time > time)return ;

	double current_command;

	if(commands_log[commands_log.size()-1].second + delay_time < time)current_command = commands_log[commands_log.size()-1].first;
	else for(int i = commands_log.size()-2; i>=0; i--)
	{
		if(commands_log[i].second + delay_time < time && commands_log[i+1].second + delay_time > time)
		{
			current_command = commands_log[i].first;
			break;
		}
	}
	if(!simplified)
	{
		for(int i=0; i<1; i++) //to change integration frequency (leave as is for 1 khz)
			this->steering_model.next(0.001, current_command, vx, ax);
		value = this->steering_model.get_wheel_angle()*3.14159/180;
	}
	else
	{
		if(current_command > value)
		{
			value = std::min(value + speed * dt, current_command);
		}
		else value = std::max(value - speed * dt, current_command);
	}
	
}

void brakeActuation::propagate(double dt, double time)
{
	if(commands_log.size() == 0)return ;
	if(commands_log[0].second + delay_time > time)return ;

	double current_command;

	if(commands_log[commands_log.size()-1].second + delay_time < time)current_command = commands_log[commands_log.size()-1].first;
	else for(int i = commands_log.size()-2; i>=0; i--)
	{
		if(commands_log[i].second + delay_time < time && commands_log[i+1].second + delay_time > time)
		{
			current_command = commands_log[i].first;
			break;
		}
	}

	if(current_command > value)
	{
		value = std::min(value + speed * dt, current_command);
	}
	else value = std::max(value - 0.5* speed * dt, current_command);
}

void motorActuation::propagate(double dt, double time)
{
	if(commands_log.size() == 0)return ;
	if(commands_log[0].second + delay_time > time)return ;

	double current_command;
	int i_sel;
	if(commands_log[commands_log.size()-1].second + delay_time < time)
	{
		current_command = commands_log[commands_log.size()-1].first;
		i_sel = -1;
	}
	else for(int i = commands_log.size()-2; i>=0; i--)
	{
		if(commands_log[i].second + delay_time <= time && commands_log[i+1].second + delay_time > time)
		{
			// std::cout<<commands_log[i].first<<" "<<commands_log[i].second<<std::endl;
			current_command = commands_log[i].first;
			i_sel = i;
			break;
		}
	}
	// std::cout<<current_command<<std::endl;
	value = current_command;
}

template<typename T>
void Delay<T>::propagate(double dt, double time)
{
	if(this->commands_log.size() == 0)return ;
	if(this->commands_log[0].second + this->delay_time > time)return ;

	double current_command;
	int i_sel;
	if(this->commands_log[this->commands_log.size()-1].second + this->delay_time < time)
	{
		current_command = this->commands_log[this->commands_log.size()-1].first;
		i_sel = -1;
	}
	else for(int i = this->commands_log.size()-2; i>=0; i--)
	{
		if(this->commands_log[i].second + this->delay_time <= time && this->commands_log[i+1].second + this->delay_time > time)
		{
			// std::cout<<this->commands_log[i].first<<" "<<this->commands_log[i].second<<std::endl;
			current_command = this->commands_log[i].first;
			i_sel = i;
			break;
		}
	}
	// std::cout<<current_command<<std::endl;
	this->value = current_command;
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


/* STEERING MODELLING */
/*
	BACKLASH:
	When gears dont touch, trer = 0, dprack = 0
	When gears touch, trer = vdfunc, dprack = ...
	SITUATIONS:
	1. c_bl = bl+ && r_m>0 -> trer = fvd, dc_bl = 0, dprack = ...r_mdt
	2. b- <= p + r_m*dt <= b+ && r_m >0 -> r_m <0

*/
SteeringState::SteeringState() : v_a(0), ang_v_motor(0), ang_v_motor_dot(0), i_a(0), i_a_dot(0), theta_gear(0), theta_gear_dot(0), p_rack(0), p_rack_dot(0), my(0), fy(0), fz(0), f_rack(0), t_rer(0), current_backlash(0), backlash_neg(-0.03), backlash_pos(0.03), i_controller_1(0), i_controller_2(0), v_out(0), i_out(0), situation(0), ticks_to_control(10), tick_n(10) // integ_freq = 1000, control_freq = 100
{
	angle_to_rack_displacement = {
		/*0*/std::make_pair(24, 25.487825),
		/*1*/std::make_pair(19.2, 20.060725), 
		/*2*/std::make_pair(14.4, 14.87209),
		/*3*/std::make_pair(9.6, 9.8376545),
		/*4*/std::make_pair(4.8, 4.8966805),
		/*5*/std::make_pair(0, 0),
		/*6*/std::make_pair(-4.8, -4.8966805),
		/*7*/std::make_pair(-9.6, -9.8376545),
		/*8*/std::make_pair(-14.4, -14.87209),
		/*9*/std::make_pair(-19.2, -20.060725), 
		/*10*/std::make_pair(-24, -25.487825)
	};

	// for(double i = -28; i<=28; i+=0.5)
	// {
	// 	theta_ref = std::min(25.487825, std::max(-25.487825, i));
	// 	std::cout<<calc_p_rack_ref()<<", ";
	// }
	// exit(0);
	log_file.open("src/6.Controls/simple_sim/data/steering_log.txt");
	log_file<<*this;
}
std::ostream &operator<<(std::ostream &out, const SteeringState &a)
{
	// out << std::setw(7) /* 0 */<<"v_a"
	// 	/* 1 */<< "\t"  << std::fixed << std::setw(10) << "ω"
	// 	/* 2 */<< "\t"  << std::fixed << std::setw(10) << "ω_dot"
	// 	/* 3 */<< "\t"  << std::fixed << std::setw(10) << "i_a"
	// 	/* 4 */<< "\t"  << std::fixed << std::setw(10) << "i_a_dot"
	// 	/* 5 */<< "\t"  << std::fixed << std::setw(10) << "θ_gear"
	// 	/* 6 */<< "\t"  << std::fixed << std::setw(10) << "θ_gear_dot"
	// 	/* 7 */<< "\t"  << std::fixed << std::setw(10) << "p_rack"
	// 	/* 8 */<< "\t"  << std::fixed << std::setw(10) << "p_rack_dot"
	// 	/* 9 */<< "\t"  << std::fixed << std::setw(10) << "backlash"
	// 	/* 10 */<< "\t"  << std::fixed << std::setw(10) << "dt"
	// 	/* 11 */<< "\t"  << std::fixed << std::setw(10) << "θ_ref_orig"
	// 	/* 12 */<< "\t"  << std::fixed << std::setw(10) << "θ_ref"
	// 	/* 13 */<< "\t"  << std::fixed << std::setw(10) << "θ_out"
	// 	/* 14 */<< "\t"  << std::fixed << std::setw(10) << "p_rack_ref"
	// 	/* 15 */<< "\t"  << std::fixed << std::setw(10) << "ω_ref"
	// 	/* 16 */<< "\t"  << std::fixed << std::setw(10) << "i_out"
	// 	/* 17 */<< "\t"  << std::fixed << std::setw(10) << "i_out_sat"
	// 	/* 18 */<< "\t"  << std::fixed << std::setw(10) << "v_out"
	// 	/* 19 */<< "\t"  << std::fixed << std::setw(10) << "v_out_sat"
	// 	/* 20 */<< "\t"  << std::fixed << std::setw(10) << "error_w"
	// 	/* 21 */<< "\t"  << std::fixed << std::setw(10) << "error_i"
	// 	/* 22 */<< "\t"  << std::fixed << std::setw(10) << "error_i_2"
	// 	/* 23 */<< "\t"  << std::fixed << std::setw(10) << "error_v"
	// 	/* 24 */<< "\t"  << std::fixed << std::setw(10) << "i_c_1"
	// 	/* 25 */<< "\t"  << std::fixed << std::setw(10) << "i_c_2"
	// 	/* 26 */<< "\t"  << std::fixed << std::setw(10) << "t_rer"
	// 	/* 27 */<< "\t"  << std::fixed << std::setw(10) << "fy"
	// 	/* 28 */<< "\t"  << std::fixed << std::setw(10) << "fz"
	// 	/* 29 */<< "\t"  << std::fixed << std::setw(10) << "my"
	// 	/* 30 */<< "\t"  << std::fixed << std::setw(10) << "sit"
	// 	<< std::endl;
	out << std::setw(7) /* 0 */<<a.v_a
		/* 1 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.ang_v_motor
		/* 2 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.ang_v_motor_dot
		/* 3 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_a
		/* 4 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_a_dot
		/* 3 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.theta_gear
		/* 6 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.theta_gear_dot
		/* 7 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.p_rack
		/* 8 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.p_rack_dot
		/* 9 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.current_backlash
		/* 10 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.dt
		/* 11 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.theta_ref_orig
		/* 12 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.theta_ref
		/* 13 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.get_wheel_angle()
		/* 14 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.p_rack_ref
		/* 13 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.w_ref
		/* 16 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_out
		/* 17 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_out_sat
		/* 18 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.v_out
		/* 19 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.v_out_sat
		/* 20 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.error_w
		/* 21 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.error_i
		/* 22 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.error_i_2
		/* 23 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.error_v
		/* 24 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_controller_1
		/* 23 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_controller_2
		/* 26 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.t_rer
		/* 27 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.fy
		/* 28 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.fz
		/* 29 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.my
		/* 30 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.situation
		<< std::endl;
}
void SteeringState::next(double dt, double theta_ref, double vx, double ax)
{
	// std::cout<<std::endl<<1;
	this->dt = dt;
	theta_ref_orig = theta_ref;
	this->theta_ref = theta_ref * 180 / 3.14159;
	this->theta_ref = std::min(25.487825, std::max(-25.487825, this->theta_ref));
	this->vx = vx;
	this->ax = ax;
	// std::cout<<std::endl<<2;
	if(tick_n == ticks_to_control)
	{
		tick_n = 0;
		p_rack_ref = calc_p_rack_ref();

		w_ref = calc_w_ref();

		i_out_sat = calc_i_ref();

		v_out_sat = calc_v_ref();
		v_a = v_out_sat;
	}
	tick_n++;
	// std::cout<<std::endl<<3;
	double dt_for_second;
	bool needs_second = 0;

	if(ang_v_motor > 0)
	{
		// std::cout<<std::endl<<4;
		if(current_backlash + ang_v_motor * this->dt <= backlash_pos)
		{
			// std::cout<<std::endl<<5;
			/* Deadband area, pushing right*/
			situation = 1;
			p_rack_dot = 0;
			t_rer = 0;
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			current_backlash = std::min(current_backlash + ang_v_motor * this->dt, backlash_pos);
		}
		else if(backlash_pos - current_backlash < 0.01 * (ang_v_motor * this->dt)) 
		{
			// std::cout<<std::endl<<6;
			/* We consider that the gears touch when pos is within 5% of the total dt-movement from the right gear*/
			/* Right side of deadband. gears kind of touch, pushing right*/
			situation = 3;
			current_backlash = backlash_pos;
			t_rer = calc_t_rer();
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			p_rack_dot = calc_p_rack_dot();
		}
		else
		{
			// std::cout<<std::endl<<7;
			situation = 2;
			this->dt = (backlash_pos - current_backlash) / ang_v_motor;
			dt_for_second = dt - this->dt;
			needs_second = true;

			p_rack_dot = 0;
			t_rer = 0;
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			current_backlash = std::min(current_backlash + ang_v_motor * this->dt, backlash_pos);
			
		}
	}
	else if(ang_v_motor<0)
	{
		// std::cout<<std::endl<<8;
		if(current_backlash + ang_v_motor * this->dt >= backlash_neg)
		{
			situation = -1;
			// std::cout<<std::endl<<9;
			/* Deadband area, pushing right*/
			p_rack_dot = 0;
			t_rer = 0;
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			current_backlash = std::max(current_backlash + ang_v_motor * this->dt, backlash_neg);
		}
		else if(current_backlash - backlash_neg < 0.01 * (- ang_v_motor * this->dt)) 
		{
			situation = -3;
			// std::cout<<std::endl<<10;
			/* We consider that the gears touch when pos is within 5% of the total dt-movement from the right gear*/
			/* Right side of deadband. gears kind of touch, pushing right*/
			current_backlash = backlash_neg;
			t_rer = -calc_t_rer();
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			p_rack_dot = calc_p_rack_dot();
		}
		else
		{
			situation = -2;
			// std::cout<<std::endl<<11;
			this->dt = (current_backlash-backlash_neg) / (-ang_v_motor);
			dt_for_second = dt - this->dt;
			needs_second = true;

			p_rack_dot = 0;
			t_rer = 0;
			i_a_dot = calc_i_a_dot();
			ang_v_motor_dot = calc_ang_v_motor_dot();
			theta_gear_dot = calc_theta_dot();
			// std::cout<<current_backlash<<" "<<backlash_neg<<" "<<this->dt<<"\t//\t";
			current_backlash = std::max(current_backlash + ang_v_motor * this->dt, backlash_neg);
			
		}
	}
	else
	{
		situation = 0;
		p_rack_dot=0;
		t_rer=0;
		i_a_dot = calc_i_a_dot();
		ang_v_motor_dot = calc_ang_v_motor_dot();
		theta_gear_dot = calc_theta_dot();
	}
	// std::cout<<std::endl<<12;
	i_a = i_a_next();
	ang_v_motor = ang_v_motor_next();
	theta_gear = theta_gear_next();
	p_rack = p_rack_next();
	log_file<<*this;
	// std::cout<<std::endl<<13;
	if(needs_second)
	{
		// std::cout<<std::endl<<14;
		this->next(dt_for_second, theta_ref, vx, ax);
	}
}
double SteeringState::get_wheel_angle()const
{
	if(p_rack == 0)return 0;
	else if(p_rack>0)
	{
		for(int i=4; i>=0; i--)
		{
			if(	p_rack >  angle_to_rack_displacement[i+1].first &&
				p_rack <= angle_to_rack_displacement[i].first)
				{
					return (p_rack - angle_to_rack_displacement[i+1].first)/(angle_to_rack_displacement[i].first - angle_to_rack_displacement[i+1].first)*angle_to_rack_displacement[i].second + (-p_rack + angle_to_rack_displacement[i].first)/(angle_to_rack_displacement[i].first - angle_to_rack_displacement[i+1].first)*angle_to_rack_displacement[i+1].second;
				}
		}
		if(p_rack >= angle_to_rack_displacement[0].first)return angle_to_rack_displacement[0].second;
	}
	else
	{

		for(int i=5; i<=10; i++)
		{
			if(	p_rack >  angle_to_rack_displacement[i+1].first &&
				p_rack <= angle_to_rack_displacement[i].first)
				{
					return (p_rack - angle_to_rack_displacement[i+1].first)/(angle_to_rack_displacement[i].first - angle_to_rack_displacement[i+1].first)*angle_to_rack_displacement[i].second + (-p_rack + angle_to_rack_displacement[i].first)/(angle_to_rack_displacement[i].first - angle_to_rack_displacement[i+1].first)*angle_to_rack_displacement[i+1].second;
				}
		}
		if(p_rack <= angle_to_rack_displacement[10].first)return angle_to_rack_displacement[10].second;
	}
}
double SteeringState::calc_t_rer()
{
	my = this->calc_my();
	fz = this->calc_fz();
	fy = my*fz;
	f_rack = 2 * fy * constants.c / constants.b;
	return (f_rack * constants.lead * constants.tr_ratio)/constants.st_efficiency;
}
double SteeringState::calc_i_a_dot()const
{
	return (v_a - constants.Ra * i_a - constants.Kv * ang_v_motor) / constants.La;
}
double SteeringState::calc_ang_v_motor_dot()const
{
	return (constants.kt * i_a - constants.r*ang_v_motor - t_rer - 0.02 * p_rack) * constants._1_J; /* TBD */
}
double SteeringState::calc_theta_dot()const
{
	return ang_v_motor * constants._1_Red;
}
double SteeringState::calc_p_rack_dot()const
{
	return constants.pinion_gear_diameter * theta_gear_dot;
}
double SteeringState::i_a_next()const
{
	return i_a + dt * i_a_dot;
}
double SteeringState::ang_v_motor_next()const
{
	return ang_v_motor + ang_v_motor_dot * dt;
}
double SteeringState::theta_gear_next()const
{
	return theta_gear + theta_gear_dot * dt ;
}
double SteeringState::p_rack_next()const
{
	return p_rack + p_rack_dot * dt;
}
double SteeringState::calc_i_ref()
{
	error_w = w_ref - ang_v_motor;
	error_i = i_out_sat - i_out;
	i_controller_1 += this->dt*(error_w + (1 / constants.kps)*error_i);
	i_out = constants.kps*error_w + constants.kis * i_controller_1 + 10 * w_ref;
	i_out_sat = std::min(constants.i_sat, std::max(-constants.i_sat, i_out));
	return i_out_sat;
}
double SteeringState::calc_v_ref()
{
	error_i_2 = i_out_sat - i_a;
	error_v = v_out_sat - v_out;
	i_controller_2 += this->dt * (error_i_2 + (1 / constants.kp)*error_v);
	v_out = constants.kp * error_i_2 + constants.ki*i_controller_2;
	v_out_sat = std::min(constants.v_sat, std::max(-constants.v_sat, v_out));
	return v_out_sat;
}
double SteeringState::calc_w_ref()
{
	double error_p = p_rack_ref - p_rack;
	w_ref = steering_pid_controller(error_p, this->dt);
	return w_ref;
}
double SteeringState::calc_p_rack_ref()const
{
	if(theta_ref == 0)return 0;
	else if(theta_ref>0)
	{
		for(int i=4; i>=0; i--)
		{
			if(	theta_ref >  angle_to_rack_displacement[i+1].second &&
				theta_ref <= angle_to_rack_displacement[i].second)
				{
					return (theta_ref - angle_to_rack_displacement[i+1].second)/(angle_to_rack_displacement[i].second - angle_to_rack_displacement[i+1].second)*angle_to_rack_displacement[i].first + (-theta_ref + angle_to_rack_displacement[i].second)/(angle_to_rack_displacement[i].second - angle_to_rack_displacement[i+1].second)*angle_to_rack_displacement[i+1].first;
				}
		}
		if(theta_ref >= angle_to_rack_displacement[0].second)return angle_to_rack_displacement[0].first;
	}
	else
	{

		for(int i=5; i<=10; i++)
		{
			if(	theta_ref >  angle_to_rack_displacement[i+1].second &&
				theta_ref <= angle_to_rack_displacement[i].second)
				{
					return (theta_ref - angle_to_rack_displacement[i+1].second)/(angle_to_rack_displacement[i].second - angle_to_rack_displacement[i+1].second)*angle_to_rack_displacement[i].first + (-theta_ref + angle_to_rack_displacement[i].second)/(angle_to_rack_displacement[i].second - angle_to_rack_displacement[i+1].second)*angle_to_rack_displacement[i+1].first;
				}
		}
		if(theta_ref <= angle_to_rack_displacement[10].second)return angle_to_rack_displacement[10].first;
	}
}
double SteeringState::calc_fz()const /* must be ffz */
{
	return (constants.m * constants.wd * constants.g - constants.h_cog*constants.m*ax/constants.wheelbase) + 0.25 * constants.P_air * constants.ClA* vx * vx;
}
double SteeringState::calc_my()const
{
	const double Fz = calc_fz();
	const double Fz0 = 1112.0554070627252;
	const double dFz = (Fz - Fz0) / Fz0;
	const double mx_max = 0.66 * (2.21891927 - 1.36151651e-07 * dFz);
	const double my_max = 0.66 * (2.46810824 - 0.21654031 * dFz);
	return my_max;
}



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


sim_node::sim_node() : Node("Simple_Simulation"), state(), constants(193.5, 250.0, 1.59, 0.66, 1.225, 2.0, 7.0, 3.9, 0.85, 0.2, 9.81, 0.275, 0.467, 4, 2, 0.079, 0.0735, 0.6, 0.025,/*steering*/ 0.000279, 0.293, 0.0525, 11783, 0.052493, 0.015152, 0.06, 0.0, 27.98, 56115, 503.36, 973, 0.005, 0.0004, 0.00004, 100, 20, 30, 24*0.9, 5.78e-5, 0.01434, 0.0508, 0.002, 0.5, 0.8, 620), global_idx(0), steering_dead_time(0.07), motor_dead_time(0.01), last_d(0), idx_of_last_lap(-1), sent(0), is_end(0), total_doo(0), steering_model(), brake_model(), motor_model()
{
	pubs_and_subs();
	parameter_load();

	steering_model.init(steering_response, get_parameter("steering_kp").as_double(), get_parameter("steering_kd").as_double(), constants, get_parameter("steering_simple_model").as_bool());
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

void sim_node::pubs_and_subs()
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
	int i=0;
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

		/* Brake Pressure and Motor Torque*/
		
		brake_press = brake_model.get_command();
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
			/*
			if (int(torques.size()) - mot_d_ticks - 1 < 0) frx_ = 0.0;
			else frx_ = torques[int(torques.size()) - mot_d_ticks - 1] * constants.gr * constants.eff / constants.R_wheel;
			ffx_ = 0.0;
			*/
			frx_ = motor_model.get_command()* constants.gr * constants.eff / constants.R_wheel;
			ffx_ = 0;
		}

		/*
		if (int(steering.size()) - st_d_ticks - 1 < 0)
		{
			d = 0;
			// std::cout<<steering.size()<<" "<<st_d_ticks<<" / ";
		}
		else
			d = steering[int(steering.size()) - st_d_ticks - 1];

		if (last_d2 + 0.0005 < d) last_d2 = last_d2 + 0.0005;
		else if (last_d2 - 0.0005 > d) last_d2 = last_d2 - 0.0005;
		*/

		last_d = steering_model.get_command();
		last_d = std::min(3.14159 * 31.2 / 180, std::max(-3.14159 * 31.2 / 180, last_d));
		state.next(dt_, frx_, ffx_, last_d);
		steering_model.propagate(1e-3, state.t*1e3, state.v_x, state.a_x);
		brake_model.propagate(1e-3, state.t*1e3);
		motor_model.propagate(1e-3, state.t*1e3);
		std::ofstream log2;
		if(steering.size()!=0)
		{
			log2.open("src/6.Controls/simple_sim/data/log2.txt", std::ios::app);
			//state.check_ellipses(log2);
			log2<<last_d<<"\t"<<steering[steering.size()-1]<<std::endl;
			log2.close();
		}

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
		// std::cout<<frx_<<" "<<ffx_<<std::endl;
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

		/* Check if we are hitting cones*/
		// state.theta, state.x, state.y
		double sin = std::sin(state.theta), cos = std::cos(state.theta);
		for(int i=0; i<seen_cones.size(); i++)
		{
			Cone cone = seen_cones[i];
			if(has_hit_cone(sin, cos, state.x, state.y, cone.x, cone.y))
			{
				seen_cones.erase(seen_cones.begin()+i);
				i--;
				total_doo++;
				RCLCPP_WARN(get_logger(), "A cone has bin hit. Total count: %d", total_doo);
			}
		}
	}

	if (global_idx % 250 == 0)
	{
		std::cout << "Lap: "<< state.lap<< "\t\t" << state.t << "\t\t" << state.v_x << std::endl;
		// std::cout << '*' << std::endl;
		//  std::cout<<"> "<<unseen_cones.size()<<" "<<seen_cones.size()<<std::endl;
		for (int i = 0; i < unseen_cones.size(); i++)
		{
			double dsq = std::pow(state.x - unseen_cones[i].x, 2) + std::pow(state.y - unseen_cones[i].y, 2);
			if (dsq < perception_range * perception_range && dsq > 4 && std::acos((std::cos(state.theta) * (-state.x + unseen_cones[i].x) + std::sin(state.theta) * (-state.y + unseen_cones[i].y)) / std::sqrt(dsq)) < (3.14159 * 105 / 180))
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
}

void sim_node::command_callback(const custom_msgs::msg::TxControlCommand::SharedPtr msg) {
	torques.push_back(msg->motor_torque_target);
	motor_model.add_command(msg->motor_torque_target, state.t*1e3);
	// motor_model.add_command(20, state.t*1e3);

	steering.push_back(msg->steering_angle_target);
	steering_model.add_command(msg->steering_angle_target, state.t*1e3);
	// steering_model.add_command(0.4*std::sin(state.t), state.t*1e3);
	// steering_model.add_command(0.3, state.t*1e3);

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
