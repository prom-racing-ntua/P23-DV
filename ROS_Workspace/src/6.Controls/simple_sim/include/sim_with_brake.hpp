#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <climits>
#include <vector>
#include <cmath>
#include <utility>
#include <string>
#include <climits>
#include <cfloat>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <iomanip>

#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/tx_system_state.hpp"
#include "custom_msgs/msg/rx_vehicle_sensors.hpp"
#include "custom_msgs/msg/rx_steering_angle.hpp"
#include "custom_msgs/msg/rx_wheel_speed.hpp"
#include "custom_msgs/msg/autonomous_status.hpp"
#include "custom_msgs/msg/mission_selection.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace sim
{
typedef std::string string;
struct Constants
{
    double m, Iz, wheelbase, c_tire, P_air, CdA, ClA, gr, eff, R_wheel, g, h_cog, wd, R_disk_f, R_disk_r, mi_disk, d_piston, La, Ra, kt, _1_J, Kv, _1_Red, back_width, back_center, pinion_gear_diameter, kps, kis, ki, dt_rack, dt_w_motor, dt_current, kp_custom, kd_custom, i_sat, v_sat, r, c, b, lead, tr_ratio, st_efficiency, kp;
    int N_front, N_rear;
    Constants() {}
    Constants(double _1, double _2, double _3, double _4, double _5, double _6, double _7, double _8, double _9, double _10, double _11, double _12, double _13, int _14, int _15, double _16, double _17, double _18, double _19, double _20, double _21, double _22, double _23, double _24, double _25, double _26, double _27, double _28, double _29, double _30, double _31, double _32, double _33, double _34, double _35, double _36, double _37, double _38, double _39, double _40, double _41, double _42, double _43, double _44, double _45):
    m(_1), Iz(_2), wheelbase(_3), c_tire(_4), P_air(_5), CdA(_6), ClA(_7), gr(_8), eff(_9), R_wheel(_10), g(_11), h_cog(_12), wd(_13), N_front(_14), N_rear(_15),  R_disk_f(_16), R_disk_r(_17), mi_disk(_18), d_piston(_19), Ra(_20), La(_21), kt(_22), _1_J(_23), Kv(_24), _1_Red(_25), back_width(_26), back_center(_27), pinion_gear_diameter(_28), kps(_29*1), kis(_30*1), ki(_31*1), dt_rack(_32), dt_w_motor(_33), dt_current(_34), kp_custom(_35*1), kd_custom(_36*1), i_sat(_37), v_sat(_38), r(_39), c(_40), b(_41), lead(_42), tr_ratio(_43), st_efficiency(_44), kp(_45*1) {}
};

struct Cone
{
    double x,y;
    int color; //0yellow 1blue 2smallorange 3bigorange
    Cone(double a, double b, int c):
        x(a), y(b), color(c) {}
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
template<typename T>
class Actuation
{
    protected:
    std::vector<std::pair<T, double> > commands_log; // command, timestamp
    double delay_time; //in ms
    double speed;
    T value; // current value
    public:
    Actuation();
    void init(double delay, double approach_speed);
    void add_command(T command, double time);
    T get_command()const;
    virtual void propagate(double dt, double time) = 0;
};


template<typename T>
class Delay: public Actuation<T>
{
    public:
    void init(double delay);
    void propagate(double dt, double time) override;
};



class SteeringState
{
    public:
    SteeringState();
    void init(Constants a, int kp, int ki, int kd, int dt, int max_out){constants = a; this->kp = kp; this->ki = ki; this->kd = kd; this->max_out = max_out;this->dt2 = dt;  steering_pid_controller.init(kp, ki, kd, dt, max_out);}
    void next(double dt, double theta_ref, double vx, double ax);
    double get_wheel_angle()const;

    private:
    Constants constants;
    double dt;
    std::vector<std::pair<double, double> > angle_to_rack_displacement;
    /* motor model */
    double v_a, ang_v_motor, ang_v_motor_dot, i_a, i_a_dot, theta_gear, theta_gear_dot;
    double p_rack, p_rack_dot;
    double my, fz, fy, f_rack, t_rer;
    double vx, ax, theta_ref, p_rack_ref, theta_ref_orig;
    double backlash_neg, backlash_pos, current_backlash;
    int situation;
    double time;
    int tick_n, ticks_to_control;
    /* pids */
    double error_w, w_ref, error_i, i_out, i_out_sat, i_controller_1;
    double error_i_2, error_v, v_out_sat, v_out, i_controller_2;
    double kp, ki, kd, dt2, max_out;
    Delay<double> ang_v_motor_data, i_a_data, p_rack_data;
    double calc_t_rer();
    double calc_i_a_dot()const;
    double calc_ang_v_motor_dot()const;
    double calc_theta_dot()const;
    double calc_p_rack_dot()const;
    double i_a_next()const;
    double ang_v_motor_next()const;
    double theta_gear_next()const;
    double p_rack_next()const;
    double calc_fz()const;
    double calc_my()const;
    double calc_i_ref();
    double calc_v_ref();
    double calc_w_ref();
    double calc_p_rack_ref()const;
    double calc_ang_from_p_rack()const;
    PID steering_pid_controller;
    std::ofstream log_file;
    friend std::ostream &operator<<(std::ostream &out, const SteeringState &a);
};



class steeringActuation: public Actuation<double>
{
    private:
    SteeringState steering_model;
    bool simplified;
    public:
    void init(double delay, double kp, double kd, Constants a, bool simplified);
    void propagate(double dt, double time, double vx, double ax);
    void propagate(double dt, double time)override {exit(69);}
};

class brakeActuation: public Actuation<double>
{
    public:
    // brakeActuation();
    void propagate(double dt, double time) override;
};

class motorActuation: public Actuation<double>
{
    public:
    // motorActuation();
    void init(double delay);
    void propagate(double dt, double time) override;
};

struct Forces {
    double ffx, frx;
};


class State
{
public:
    double v_x, v_y, a_x, a_y, r, x, y, theta;
    double sa_f, sa_r, s, t, ffy, fry, ffz, frz, frx, ffx, d;
    int lap;
    State() : v_x(0), v_y(0), a_x(0), a_y(0), r(0), x(0), y(0), theta(0), sa_f(0), sa_r(0), s(0), t(0), fry(0), ffy(0), lap(0), ffz(0), frz(0) {}
    void init(Constants a, bool b){constants=a;simplified=b;}
    static std::pair<double, double> get_mx_my(double vx, double ax);
    void next(double dt, double Frx, double Ffx, double delta);

private:
    Constants constants;
    bool simplified;
    double calc_sa_r(double v_y, double r, double v_x) const;
    double calc_sa_f(double v_y, double r, double v_x, double d) const;
    double calc_fry(double sa_r, double frz) const;
    double calc_ffy(double sa_f, double frz) const;
    double calc_ffz(double v_x) const;
    double calc_frz(double v_x) const;
    double calc_lr(bool simplified = 1, double a_x = 0) const;
    double calc_lf(double lr) const;
    double calc_drag(double v_x) const;
    double calc_roll(double fz)const;
    double v_x_next(double v_x, double a_x, double dt) const;
    double v_y_next(double v_y, double a_y, double dt) const;
    double calc_a_x(double frx, double ffx, double f_drag, double f_roll, double ffy, double d, double v_y, double r) const;
    double calc_a_y(double fry, double ffx, double ffy, double d, double v_x, double r) const;
    double r_next(double ffy, double ffx, double d, double fry, double r, double dt) const;
    double theta_next(double r, double theta, double dt)const;
    double x_next(double v_x, double theta, double v_y, double x, double dt) const;
    double y_next(double v_x, double theta, double v_y, double y, double dt) const;
    double s_next(double s, double v_x, double v_y, double a_x, double a_y, double dt) const;
};

class sim_node: public rclcpp::Node
{
    public:
    sim_node();
    ~sim_node() {}
    private:
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pub_pose;
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr pub_map;
    rclcpp::Publisher<custom_msgs::msg::WaypointsMsg>::SharedPtr pub_way;
    rclcpp::Publisher<custom_msgs::msg::VelEstimation>::SharedPtr pub_vel;
    rclcpp::Publisher<custom_msgs::msg::TxSystemState>::SharedPtr pub_syst;
    rclcpp::Publisher<custom_msgs::msg::RxVehicleSensors>::SharedPtr pub_sens;
    rclcpp::Publisher<custom_msgs::msg::RxSteeringAngle>::SharedPtr pub_steer;
    rclcpp::Publisher<custom_msgs::msg::RxWheelSpeed>::SharedPtr pub_wheel;
    rclcpp::Publisher<custom_msgs::msg::AutonomousStatus>::SharedPtr pub_aut;
    rclcpp::Publisher<custom_msgs::msg::MissionSelection>::SharedPtr pub_miss;
    rclcpp::Subscription<custom_msgs::msg::TxControlCommand>::SharedPtr sub_comm;

    State state;
    Constants constants;

    std::vector<Cone> seen_cones;
    std::vector<Cone> unseen_cones;
    std::vector<double> torques;
    std::vector<double> steering;

    void timer_callback();
    void command_callback(const custom_msgs::msg::TxControlCommand::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    bool lap_change()const;

    void pubs_and_subs();
    void parameter_load();

    // Actuator modelling
    steeringActuation steering_model;
    double steering_response, steering_velocity;

    brakeActuation brake_model;
    double brake_response, brake_velocity;

    motorActuation motor_model;
    double motor_response;

    double perception_range;

    std::ofstream log;
    long long int global_idx;
    double steering_dead_time;
    double motor_dead_time;
    double brake_dead_time;
    int st_d_ticks;
    int mot_d_ticks;
    int br_d_ticks;
    double last_d, last_d2;
    int idx_of_last_lap;
    int sent;
    int discipline;
    int is_end;
    float brake_press;
    int total_doo;
};
}