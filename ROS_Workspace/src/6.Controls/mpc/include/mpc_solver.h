#include <stdio.h>
#include "../src/FORCESNLPsolver/include/FORCESNLPsolver.h"
#include "../src/FORCESNLPsolver/include/FORCESNLPsolver_memory.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <unistd.h>
#include <array>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>
#include <unistd.h>
#include <iterator>
#include <chrono>
#include <iomanip>
#include <functional>
#include <memory>
#include "arc_length_spline.h"
#include "read_track.h"
#include "Eigen/Dense"

extern "C" {
#include <stdio.h>
#include "../src/FORCESNLPsolver/include/FORCESNLPsolver.h"
#include "../src/FORCESNLPsolver/include/FORCESNLPsolver_memory.h"

/* AD tool - FORCESPRO interface */
extern solver_int32_default FORCESNLPsolver_adtool2forces(FORCESNLPsolver_float *x,       /* primal vars                                         */
                                                      FORCESNLPsolver_float *y,       /* eq. constraint multiplers                           */
                                                      FORCESNLPsolver_float *l,       /* ineq. constraint multipliers                        */
                                                      FORCESNLPsolver_float *p,       /* parameters                                          */
                                                      FORCESNLPsolver_float *f,       /* objective function (scalar)                         */
                                                      FORCESNLPsolver_float *nabla_f, /* gradient of objective function                      */
                                                      FORCESNLPsolver_float *c,       /* dynamics                                            */
                                                      FORCESNLPsolver_float *nabla_c, /* Jacobian of the dynamics (column major)             */
                                                      FORCESNLPsolver_float *h,       /* inequality constraints                              */
                                                      FORCESNLPsolver_float *nabla_h, /* Jacobian of inequality constraints (column major)   */
                                                      FORCESNLPsolver_float *hess,    /* Hessian (column major)                              */
                                                      solver_int32_default stage,     /* stage number (0 indexed)                            */
                                                      solver_int32_default iteration, /* iteration number of solver                          */
                                                      solver_int32_default threadID /* Id of caller thread 								 */);
}

using namespace path_planning;

namespace mpc {

struct velocity_data {
    double velocity_x;
    double velocity_y;
    double yaw_rate;
};

struct pose_data {
    double x;
    double y;
    double theta;
};

struct MuConstraints {
    double mx_max;
    double my_max;
};

struct NormalForces {
    double Ffz;
    double Frz;
};

struct SlipAngles{
    double saf;
    double sar;
};

struct output_data {
    int speed_target;
    int speed_actual;
    float motor_torque_target;
    float steering_angle_target;
    float brake_pressure_target;
};

struct center {
    float x;
    float y;
};

struct start {
    float x;
    float y;
};

class MpcSolver {
    public:
        constexpr static int X_SIZE = 9;
        constexpr static int U_SIZE = 3;
        constexpr static int Z_SIZE = X_SIZE + U_SIZE;
        int lookahead_;
        double X[X_SIZE];
        double U[U_SIZE];
        NormalForces getFz(const double X[X_SIZE]);
        MuConstraints getEllipseParams(const double &Fz);
        SlipAngles getSlipAngles(const double X[X_SIZE]);
        double getFy(double Fz, double sa);
        void getF(double X[X_SIZE],double U[U_SIZE], double (&kappa)[X_SIZE]);
        void Integrator();
        void writeParamsKnown(int global_int);
        void writeLookaheadArray1();
        void writeLookaheadArray2();
        void writeParamsUnknown();
        void Initialize_all_local();
        void UpdateFromLastIteration();
        PointsData getSplineDataLocal(std::vector<double>parameters);
        void generateFirstPoint();
        void generateFirstPointUnknown();
        int callSolver(int global_int);
        void generateOutput();
        void copyToParameters();
        void updateSkidpadSpline(int lap_counter);
        void customLapCounter();
        void generateTrackConfig();
        void generateFinishFlag(int lap_counter);
        void updateSplineParameters(std::string txt_file);
        void checkReliability();
        std::string mission_;
        std::string midpoints_txt_;
        std::vector<double> error_ver;
        std::vector<double> error_eucl;
        std::vector<double> s_vector;
        std::vector<double> ds_vector;
        std::vector<double> u_first_pass;
        std::vector<double> u_second_pass;
        std::vector<double> u_third_pass;
        std::vector<double> u_final_pass;
        int global_int_=0;
        center center_point;
        center start_point;
        int lap_counter=0;
        int lap_counter_official=0;
        bool lap_lock;
        bool finish_flag=0;
        bool brake_flag=0;
        bool braking_manouvre;
        double steer_last;
        float emergency_forward_;
        int emergency_counter=0;
        int ellipse_counter=0;
        int lap_counter_slam_;
        bool emergency;
        float sol;
        float dt;
        int closest_index_eucl;
        int closest_index_ver;
        float v_limit_;
        float dist_eucl;
        float dist_ver;
        float distance_safe_;
        velocity_data vel_struct;
        pose_data pose_struct;
        output_data output_struct;
        float s_interval_;
        float spline_resolution;
        float F_init;
        float s_space_max;
        float s_space_min;
        int exitflag = 0;
        int total_laps_;
        PointsData params_array;
        PointsData whole_track;
        FORCESNLPsolver_mem *mem;
        bool known_track_;
        bool simulation_;
        int search_window_;
        path_planning::ArcLengthSpline *spline_final;
        MpcSolver();
        ~MpcSolver();
    private:         
        double custom_max(double aa, double bb);
        double custom_min(double aa, double bb);
        std::vector<double> s_array_final;   
        const int SIM = 1000;
        const int SPLINE_RES = 2000;
        // double s_array_final[LOOKAHEAD];
        int track_resolution;
        double X2[X_SIZE];
        double X3[X_SIZE];
        double X4[X_SIZE];
        //set physical constants
        const double wb = 1.590;
        const double wd_front = 0.467;
        const double l_f = wb*(1-wd_front);
        const double l_r = wb*wd_front;
        const double CdA = 2.0; // for drag (updated)
        const double ClA = 7.0; // for downforce
        const double p_air = 1.225;
        const double h_cog = 0.27;
        const double gr = 3.9;
        const double Rw = 0.2;
        const double m = 190.0; // mass of the car
        const double g = 9.81;
        const double Iz = 110.0;
        const double ts = 0.025;
        const double fr_par = 0.03;
        const double wing = 0.874;
        //for dynamic model
        //compute l_a
        const double umin=4.0;
        const double umax=7.0;
        //ellipse params
        const double eff=0.85;
        //useful arrays
        int return_val = 0;
        double k1[X_SIZE];
        double k2[X_SIZE];
        double k3[X_SIZE];
        double k4[X_SIZE];
        clock_t st_params, end_params;
        FORCESNLPsolver_params params;
        FORCESNLPsolver_info info;
        FORCESNLPsolver_output output;
        FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
};
} //namespace mpc