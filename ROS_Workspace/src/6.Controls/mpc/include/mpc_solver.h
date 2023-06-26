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

class MpcSolver {
    public:
        constexpr static int X_SIZE = 9;
        constexpr static int U_SIZE = 3;
        constexpr static int Z_SIZE = X_SIZE + U_SIZE;
        constexpr static int LOOKAHEAD = 30; 
        int lookahead_;
        double X[X_SIZE];
        double U[U_SIZE];
        double s_array_final[LOOKAHEAD];
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
        PointsData getSplineDataLocal(double parameters[LOOKAHEAD]);
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
        std::string mission_;
        std::string midpoints_txt_;
        std::vector<double> error_ver;
        std::vector<double> error_eucl;
        std::vector<double> s_vector;
        std::vector<double> ds_vector;
        center center_point;
        int lap_counter=0;
        bool lap_lock;
        bool finish_flag=0;
        bool braking_manouvre;
        float emergency_forward_;
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
        int total_laps_;
        PointsData params_array;
        PointsData whole_track;
        FORCESNLPsolver_mem *mem;
        std::vector<double> X_spl = std::vector<double>(LOOKAHEAD);
        std::vector<double> Y_spl = std::vector<double>(LOOKAHEAD);
        std::vector<double> tang_spl = std::vector<double>(LOOKAHEAD);
        std::vector<double> curv_spl = std::vector<double>(LOOKAHEAD);
        bool known_track_;
        bool simulation_;
        int search_window_;
        path_planning::ArcLengthSpline *spline_final;
        MpcSolver();
        ~MpcSolver();
    private:         
        double custom_max(double aa, double bb);
        double custom_min(double aa, double bb);   
        const int SIM = 1000;
        const int SPLINE_RES = 2000;
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
        const double gr = 3.9;
        const double sr = 3.17;
        const double Rw = 0.2;
        const double m = 190.0; // mass of the car
        const double g = 9.81;
        const double Iz = 110.0;
        const double ts = 0.025;
        //for dynamic model
        const double B=-8.266;
        const double C=1.456;
        const double C_tire=0.66;
        const double D = 1739.47;
        const double cs=-16419.31;
        //compute l_a
        const double umin=4.0;
        const double umax=7.0;
        //ellipse params
        const double a=1.46;
        const double b=1.62;
        const double eff=0.85;
        //useful arrays
        int return_val = 0;
        int exitflag = 0;
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