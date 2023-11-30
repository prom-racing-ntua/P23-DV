#include "sim_actuator_modelling.hpp"
#include <random>
namespace sim
{
void steeringActuation::init(double delay, double kp, double kd, Constants a, bool simplified, double integration_frequency)
{
	this->steering_model.init(a, kp, 0, kd, 0.01, 3000);
	delay_time = delay;
	this->simplified = simplified;
	this->integration_frequency = integration_frequency; 
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
		for(int i=0; i<int(integration_frequency/1000); i++) //to change integration frequency (leave as is for 1 khz) MUST CHANGE CONTROL FREQ IN MODEL
			this->steering_model.next(1/integration_frequency, current_command, vx, ax);
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



void motorActuation::init(double delay) {this->Actuation<double>::init(delay, 0.0);}

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
}