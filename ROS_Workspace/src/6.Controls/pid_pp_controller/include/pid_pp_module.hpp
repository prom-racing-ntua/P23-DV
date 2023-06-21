#include <climits>
#include <vector>
#include <cmath>
#include <utility>
#include <string>
#include <climits>
#include <cfloat>
#include <iostream>
#include <fstream>

#include "arc_length_spline.h"

namespace pid_pp
{
    class Point
    {
    private:
        double x_priv;
        double y_priv;

    public:
        bool error;
        Point();
        Point(double a, double b);
        double x() const;
        double y() const;
        //friend Point operator+(const Point &a, const Point &b);
        //friend Point operator-(const Point &a, const Point &b);
        //friend Point operator*(double a, const Point &b);
        static Point midpoint(const Point &a, const Point &b);
        static double distance(const Point &a, const Point &b);
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
        ~PID();                                  // stats
    };

    class PurePursuit
    {
    private:
        double lookahead_min;    // ld clip min
        double lookahead_max;    // ld clip max
        double velocity_min;      // start of linear amping
        double velocity_max;     // end of linear amping
        double wheelbase;        // in m
        double emergency_factor; // factor by which to scale down the lookahead distance in case of an emergency manouevre

        double request_sum;
        int total_requests;

    public:
        PurePursuit();
        void init(double ld_min, double ld_max, double v_min, double v_max, double wb, double emergency_factor);
        double operator()(const Point &target, double theta, double minimum_radius) const; // target coords should be in the reference frame of the rear axle
        double lookahead(double velocity, bool emergency = 0) const;
        ~PurePursuit();
    };

    class Model
    {
    public:
        double m; // mass
        double g;
        double wb;      // wheelbase
        double wd;      // weightdistribution
        double h_cog;   // height of center of gravity
        double p_air;   // air density
        double cd_A;    // drag coefficient
        double cl_A;    // lift coefficient
        double gr;      // gear ratio
        double R_wheel; // wheel radius
        double eff;     // motor effectiveness
        double Fz0;
        double C_tire;
        double max_positive_torque;
        double max_positive_force;
        double max_negative_torque;
        double max_negative_force;
        double min_wd;

        Model() {}
        Model(double mass, double grav, double wheelbase, double weight_distribution, double h_center_og, double air_density, double drag_coeff, double lift_coeff, double gear_ratio, double wheel_radius, double m_effectiveness, double Fz_0, double c_tire, double max_p_torque, double max_n_torque, double min_wd);

        double mx_max(double Fz) const;
        double my_max(double Fz) const;
        double Fz_calc(std::string type, bool aero, bool moved_cog, double vx = 0, double wd = 0) const;
        double Torque(double F) const;
    };

    class SplinePoint // the way the spline points are saved on the velocity profile
    {
    private:
        Point position_priv;
        double s_priv;
        double phi_priv;
        double k_priv;
        double target_speed_priv;

    public:
        Point position() const;
        double s() const;
        double phi() const;
        double k() const;
        void set_target_speed(double v);
        double target_speed()const;
        SplinePoint();
        SplinePoint(Point pos, double s, double phi, double k);
    };

    class VelocityProfile
    {
    public:
        VelocityProfile() : model(nullptr), spline_samples(nullptr) {}
        VelocityProfile(path_planning::ArcLengthSpline &spline, double max_speed, int samples_per_meter, const Model &model, double initial_speed, bool is_end, bool is_first_lap);
        std::pair<double, double> operator()(const Point &position, double theta) ; // returns target velocity and cross-track error
        Point get_target_point(double ld, const Point &position, double min_radius, double theta) const;
        ~VelocityProfile();
        Point get_last_projection()const;

    private:
        const Model *model;                                            // reference to the common Model object
        double max_speed;                                              // hard speed limiter
        int samples_per_meter;                                         // resolution of spline sampling
        SplinePoint *spline_samples;                                   // will be size spline_length*samples_per_meter
        int last_visited_index;                                        // saves last visited i so as to only consider projection further along the track
        double total_length;                                           // spline_length
        int get_projection(const Point &position, double theta) const; // returns index of projection
        int max_idx;
        bool unknown;
        void solve_profile(int resolution, double initial_speed, bool is_end);
    };

}