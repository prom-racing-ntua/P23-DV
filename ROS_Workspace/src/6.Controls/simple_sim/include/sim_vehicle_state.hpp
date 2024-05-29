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
struct Forces {
    double ffx, frx;
};

typedef std::string string;
struct Constants
{
    double m, Iz, wheelbase, c_tire, P_air, CdA, ClA, gr, eff, R_wheel, g, h_cog, wd, R_disk_f, R_disk_r, mi_disk, d_piston, La, Ra, kt, _1_J, Kv, _1_Red, back_width, back_center, pinion_gear_diameter, kps, kis, ki, dt_rack, dt_w_motor, dt_current, kp_custom, kd_custom, i_sat, v_sat, r, c, b, lead, tr_ratio, st_efficiency, kp;
    int N_front, N_rear;
    Constants() {}
    Constants(double _1, double _2, double _3, double _4, double _5, double _6, double _7, double _8, double _9, double _10, double _11, double _12, double _13, int _14, int _15, double _16, double _17, double _18, double _19, double _20, double _21, double _22, double _23, double _24, double _25, double _26, double _27, double _28, double _29, double _30, double _31, double _32, double _33, double _34, double _35, double _36, double _37, double _38, double _39, double _40, double _41, double _42, double _43, double _44, double _45):
    m(_1), Iz(_2), wheelbase(_3), c_tire(_4), P_air(_5), CdA(_6), ClA(_7), gr(_8), eff(_9), R_wheel(_10), g(_11), h_cog(_12), wd(_13), N_front(_14), N_rear(_15),  R_disk_f(_16), R_disk_r(_17), mi_disk(_18), d_piston(_19), La(_20), Ra(_21), kt(_22), _1_J(_23), Kv(_24), _1_Red(_25), back_width(_26), back_center(_27), pinion_gear_diameter(_28), kps(_29*1), kis(_30*1), ki(_31*1), dt_rack(_32), dt_w_motor(_33), dt_current(_34), kp_custom(_35*1), kd_custom(_36*1), i_sat(_37), v_sat(_38), r(_39), c(_40), b(_41), lead(_42), tr_ratio(_43), st_efficiency(_44), kp(_45*1) 
    {
    }
};

struct Cone
{
    double x,y;
    int color; //0yellow 1blue 2smallorange 3bigorange
    Cone(double a, double b, int c):
        x(a), y(b), color(c) {}
};

// class Model
// {
//     public:
//         double x, y, vx, vy, ax, ay, theta, r;
//         void init(Constants vehicle_constants) = 0;
//         void next(double dt, double bp, double delta, double trq_rl, double trq_rr, double trq_fl = 0, double trq_fr = 0) = 0;
// };


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


}
