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
    double m, Iz, wheelbase, c_tire, P_air, CdA, ClA, gr, eff, R_wheel, g, h_cog, wd;
    Constants() {}
    Constants(double _1, double _2, double _3, double _4, double _5, double _6, double _7, double _8, double _9, double _10, double _11, double _12, double _13):
    m(_1), Iz(_2), wheelbase(_3), c_tire(_4), P_air(_5), CdA(_6), ClA(_7), gr(_8), eff(_9), R_wheel(_10), g(_11), h_cog(_12), wd(_13) {}
};

struct Cone
{
    double x,y;
    int color; //0yellow 1blue 2smallorange 3bigorange
    Cone(double a, double b, int c):
        x(a), y(b), color(c) {}
};

class State
{
public:
    double v_x, v_y, a_x, a_y, r, x, y, theta;
    double sa_f, sa_r, s, t, ffy, fry, ffz, frz, frx, d;
    int lap;
    State() : v_x(0), v_y(0), a_x(0), a_y(0), r(0), x(0), y(0), theta(0), sa_f(0), sa_r(0), s(0), t(0), fry(0), ffy(0), lap(0), ffz(0), frz(0) {}
    void init(Constants a, bool b){constants=a;simplified=b;}
    void check_ellipses(std::ostream &out)const;
    void next(double dt, double Frx, double delta);

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
    double calc_a_x(double frx, double f_drag, double f_roll, double ffy, double d, double v_y, double r) const;
    double calc_a_y(double fry, double ffy, double d, double v_x, double r) const;
    double r_next(double ffy, double d, double fry, double r, double dt) const;
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

    std::ofstream log;
    long long int global_idx;
    double steering_dead_time;
    double motor_dead_time;
    int st_d_ticks;
    int mot_d_ticks;
    double last_d;
    int idx_of_last_lap;
    int sent;
    int discipline;
    int is_end;

};
}