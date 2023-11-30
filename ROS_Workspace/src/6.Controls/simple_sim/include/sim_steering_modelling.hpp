#include "sim_vehicle_state.hpp"
namespace sim
{
template<typename T>
class Actuation
{
    protected:
    std::vector<std::pair<T, double> > commands_log; // command, timestamp
    double delay_time; //in ms
    double speed;
    T value; // current value
    public:
    Actuation(): delay_time(0), speed(0), value(0) {}
    void init(double delay, double approach_speed)
    {
        delay_time = delay;
        speed = approach_speed;
    }
    void add_command(T command, double time)
    {
	    commands_log.push_back(std::make_pair(command, time)); 
    }
    T get_command()const {return value;}
    T get_last_input()const {return (commands_log.size()!=0?commands_log[commands_log.size()-1].first:T());}
    virtual void propagate(double dt, double time) = 0;
};


template<typename T>
class Delay: public Actuation<T>
{
    public:
    void init(double delay);
    void propagate(double dt, double time) override;
};


class PID
{
private:
    int Kp; // proportional gain
    int Ki; // integral gain
    int Kd; // derivative gain
    double dt;
    int dampener;
    int proportional_dampener;
    int integral_dampener;
    int derivative_dampener;
    double error_integral;
    double last_error;
    double filter(double value, int dampener);

    double request_sum; // in kN
    int total_requests;

public:
    PID(); // Default Constructor
    void init(int kp, int ki, int kd, double dt, int damp, int integ_damp = INT_MAX, int prop_damp = INT_MAX, int der_damp = INT_MAX);
    double operator()(double error);            // main operation. Get controller output as object_name(error)
    double operator()(double error, double dt); // same but also provides different dt than one from constructor
    void flush_error();                      // sets error integral to 0
};


class square_wave
{
	public:
	double time;
	int revs;
	double period;
	double duty_cycle;
	double avg;
	double amplitude;
	double prev_val;
	square_wave(double T, double DT, double AVG, double A):duty_cycle(DT), avg(AVG), amplitude(A), period(T), revs(0) {}
	double operator ()(double dt)
	{
        // std::cout<<time<<" "<<period<<" "<<std::fmod(time, period)<<" "<<period * duty_cycle<<std::endl;
		if(std::fmod(time, period) < period * duty_cycle)
		{
			time += dt;
			return avg + amplitude;
		}
		else
		{
			time += dt;
			return avg - amplitude;
		}
	}
};

class SteeringState
{
    public:
    SteeringState();
    void init(Constants a, int kp, int ki, int kd, double dt, int max_out){constants = a; this->kp = kp; this->ki = ki; this->kd = kd; this->max_out = max_out;this->dt2 = dt;  steering_pid_controller.init(kp, ki, kd, dt, max_out);}
    void next(double dt, double theta_ref, double vx, double ax);
    double get_wheel_angle()const;

    private:
    Constants constants;
    double dt;
    double time;

    std::vector<std::pair<double, double> > angle_to_rack_displacement;
    /* motor model */
    double v_a, ang_v_motor, ang_v_motor_dot, i_a, i_a_dot, theta_gear, theta_gear_dot;
    double p_rack, p_rack_dot;
    double my, fz, fy, f_rack, t_rer;
    double vx, ax, theta_ref, p_rack_ref, theta_ref_orig;
    double backlash_neg, backlash_pos, current_backlash;
    int situation;
    int tick_n, ticks_to_control;
    /* pids */
    double error_w, w_ref, error_i, i_out, i_out_sat, i_controller_1;
    double error_i_2, error_v, v_out_sat, v_out, i_controller_2;
    double kp, ki, kd, dt2, max_out;
    Delay<double> ang_v_motor_data, i_a_data, p_rack_data;

    double calc_t_rer();                // Calculates resistance torque 
    double calc_i_a_dot()const;         // Calculates current derivative
    double calc_ang_v_motor_dot()const; // Calculates motor angular velocity derivative
    double calc_theta_dot()const;       // Calculates gearbox position derivative
    double calc_p_rack_dot()const;      // Calculates steering rack position derivative
    double i_a_next()const;             // Calculates next current value
    double ang_v_motor_next()const;     // Calculates next angular velocity value
    double theta_gear_next()const;      // Calculates next gearbox position value
    double p_rack_next()const;          // Calculates next steering rack position value
    double calc_fz()const;              // Calculates normal force
    double calc_my()const;
    double calc_w_ref();    // Rack position Controller (input: rack position; output: angular velocity reference)              // Calculates friction coefficient
    double calc_i_ref();    // Speed controller   (input: angular velocity;  output: current reference)
    double calc_v_ref();    // Current Controller (input: current; output: applied voltage) 
    double calc_p_rack_ref()const;      // Calculates rack position reference from requested angle 

    PID steering_pid_controller;
    std::ofstream log_file;
    friend std::ostream &operator<<(std::ostream &out, const SteeringState &a);
};


} // namespace sim
