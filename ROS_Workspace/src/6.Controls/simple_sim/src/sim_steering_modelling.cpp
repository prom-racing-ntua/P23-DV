#include "sim_steering_modelling.hpp"
namespace sim
{
template<typename T>
void Delay<T>::init(double delay) {this->Actuation<T>::init(delay, 0.0);}

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
			current_command = this->commands_log[i].first;
			i_sel = i;
			break;
		}
	}
	// std::cout<<time<<" "<<this->delay_time<<" "<<this->commands_log[this->commands_log.size()-1].first<<" "<<this->commands_log[this->commands_log.size()-1].second<<" "<<current_command<<" "<<i_sel<<" "<<this->commands_log.size()<<std::endl;
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
	// std::cout << "Error: " << error << ". Correction: " << correction << ". P: "<< proportional <<" "<<integral<<" "<<derivative<<" "<<proportional + derivative + integral<<std::endl;
	
	return correction;
}
double PID::operator()(double error, double dt)
{
	error_integral += error * dt;
	double proportional = this->filter(error * Kp, proportional_dampener);
	double integral = this->filter(error_integral * Ki, integral_dampener);
	double derivative = (last_error == -1) ? 0 : (this->filter((error - last_error) / dt, derivative_dampener));

	double correction = this->filter(proportional + integral + derivative, dampener);
	// std::cout << "Error: " << error << ". Correction: " << correction << ". P: "<< proportional <<" "<<integral<<" "<<derivative<<" "<<proportional + derivative + integral<<std::endl;
	std::cout<<error<<" "<<last_error<<" "<<derivative<<std::endl;
	request_sum += correction / 1000;
	total_requests++;
	last_error = error;

	return correction;
}
void PID::flush_error()
{
	error_integral = 0;
}

SteeringState::SteeringState() : v_a(0), ang_v_motor(0), ang_v_motor_dot(0), i_a(0), i_a_dot(0), theta_gear(0), theta_gear_dot(0), p_rack(0), p_rack_dot(0), my(0), fy(0), fz(0), f_rack(0), t_rer(0), current_backlash(0), backlash_neg(-0.03), backlash_pos(0.03), i_controller_1(0), i_controller_2(0), v_out(0), i_out(0), situation(0), ticks_to_control(1000), tick_n(1000)// integ_freq = 1000, control_freq = 100
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

	ang_v_motor_data.init(0.4); // value in ms => delay = 4e5 s
	i_a_data.init(0.06);
	p_rack_data.init(5);

	log_file.open("src/6.Controls/simple_sim/data/steering_log.txt");
	log_file<<*this;
}
std::ostream &operator<<(std::ostream &out, const SteeringState &a)
{

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
		/* 15 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.w_ref
		/* 16 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.i_out_sat
		/* 17 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.t_rer
		/* 19 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.situation
		/* 20 */<< "\t"  << std::fixed << std::setprecision(3) << std::setw(10) << a.time
		<< std::endl;
}

void SteeringState::next(double dt, double theta_ref, double vx, double ax)
{
	this->dt = dt;
	theta_ref_orig = theta_ref;	//retains initial rad value
	this->theta_ref = theta_ref * 180 / 3.14159;	//converts to degrees
	this->theta_ref = std::min(25.487825, std::max(-25.487825, this->theta_ref));	//clips input to accepted values
	this->vx = vx;	//used to calc fz (aero)
	this->ax = ax;	// 	   - / / -    (weight transfer)

	if(tick_n == ticks_to_control) // Displacement PID should have 100Hz control frequency
	{
		log_file<<*this;
		tick_n = 0;

		p_rack_ref = calc_p_rack_ref();	//converts to steering rack displacement reference according to the table in the constructor
		w_ref = calc_w_ref(); //wrapper for discplacement PID
	}
	tick_n++;

	i_out_sat = calc_i_ref(); // speed controller
	v_out_sat = calc_v_ref(); // current controller
	v_a = v_out_sat;

	/*
		BACKLASH MODELLING
		Module discerns 7 cases, depending on the relative position of the gears (current_backlash), and its predicted movement d(current_backlash) = ang_v_motor * dt:
		-> Positive angular velocity
			-> Gears don't touch and won't touch after step
				: No resistance
				: No rack movement
			-> Gears touch already
				: Resistance according to VD Function
				: Rack movement
				: No relative gear position change (d(current_backlash) = 0)
			-> Gears don't touch, but will touch during the step
				: Step is divided into substeps. 
				: This iteration executes the movement until the gears touch (dt1)
				: In the end of the function, the function gets called to execute the rest of the step
				: dt2 = dt - dt1
		-> Negative angular velocity
			... The same 3 cases as above but mirrored
		-> Zero angular velocity
			: No movement
	*/

	double dt_for_second;
	bool needs_second = 0;

	if(ang_v_motor > 0)
	{
		if(current_backlash + ang_v_motor * this->dt <= backlash_pos)
		{
			situation = 1; // Deadband area, pushing right
			
			p_rack_dot = 0;
			t_rer = 0;
			current_backlash = std::min(current_backlash + ang_v_motor * this->dt, backlash_pos);
		}
		else if(backlash_pos - current_backlash < 0.01 * (ang_v_motor * this->dt)) 
		{
			/* We consider that the gears touch when pos is within 5% of the total dt-movement from the right gear*/
			situation = 3; // Right side of deadband. gears kind of touch, pushing right
			
			p_rack_dot = calc_p_rack_dot();
			t_rer = calc_t_rer();
			current_backlash = backlash_pos;
		}
		else
		{
			situation = 2; // Intermediate step
			this->dt = (backlash_pos - current_backlash) / ang_v_motor;
			dt_for_second = dt - this->dt;
			needs_second = true;

			p_rack_dot = 0;
			t_rer = 0;
			current_backlash = std::min(current_backlash + ang_v_motor * this->dt, backlash_pos);
		}
	}
	else if(ang_v_motor<0)
	{
		if(current_backlash + ang_v_motor * this->dt >= backlash_neg)
		{
			situation = -1;	//Deadband area, pushing right

			p_rack_dot = 0;
			t_rer = 0;
			current_backlash = std::max(current_backlash + ang_v_motor * this->dt, backlash_neg);
		}
		else if(current_backlash - backlash_neg < 0.01 * (- ang_v_motor * this->dt)) 
		{
			/* We consider that the gears touch when pos is within 5% of the total dt-movement from the right gear*/
			situation = -3;	// Right side of deadband. gears kind of touch, pushing right

			current_backlash = backlash_neg;
			t_rer = -calc_t_rer();
			p_rack_dot = calc_p_rack_dot();
		}
		else
		{
			situation = -2;	// intermediate situation
			this->dt = (current_backlash-backlash_neg) / (-ang_v_motor);
			dt_for_second = dt - this->dt;
			needs_second = true;

			p_rack_dot = 0;
			t_rer = 0;
			current_backlash = std::max(current_backlash + ang_v_motor * this->dt, backlash_neg);
		}
	}
	else
	{
		situation = 0;
		p_rack_dot=0;
		t_rer=0;
	}
	
	// Calculate Derivatives
	i_a_dot = calc_i_a_dot();
	ang_v_motor_dot = calc_ang_v_motor_dot();
	theta_gear_dot = calc_theta_dot();

	// Calculate next values (execute step)
	i_a = i_a_next();
	ang_v_motor = ang_v_motor_next();
	theta_gear = theta_gear_next();
	p_rack = p_rack_next();

	// Propagate Delays
	i_a_data.propagate(this->dt, time*1e3);
	ang_v_motor_data.propagate(this->dt, time*1e3);
	p_rack_data.propagate(this->dt, time*1e3);

	// Add calculated data to Delay modules
	i_a_data.add_command(i_a, time*1e3);
	ang_v_motor_data.add_command(ang_v_motor, time*1e3);
	p_rack_data.add_command(p_rack, time * 1e3);


	time += this->dt;

	if(needs_second) // executes second substep, if deemed necessary
	{
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
	return (constants.kt * i_a - constants.r*ang_v_motor - t_rer) * constants._1_J; 
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
	error_w = w_ref - ang_v_motor_data.get_command();
	error_i = i_out_sat - i_out;
	i_controller_1 += this->dt*(error_w + (1 / constants.kps)*error_i);
	i_out = constants.kps*error_w + constants.kis * i_controller_1 + 10 * w_ref;
	i_out_sat = std::min(constants.i_sat, std::max(-constants.i_sat, i_out));
	return i_out_sat;
}
double SteeringState::calc_v_ref()
{
	error_i_2 = i_out_sat - i_a_data.get_command();
	error_v = v_out_sat - v_out;
	double ic2_dot = (error_i_2 + error_v / constants.kp)*constants.ki;
	i_controller_2 += ic2_dot * dt;
	v_out = i_controller_2 + error_i_2 * constants.kp;
	v_out_sat = std::min(constants.v_sat, std::max(-constants.v_sat, v_out)); 	
	
	return v_out_sat;

}
double SteeringState::calc_w_ref()
{
	double error_p = p_rack_ref - p_rack_data.get_command();
	w_ref = steering_pid_controller(error_p);
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
double SteeringState::calc_fz()const 
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
}