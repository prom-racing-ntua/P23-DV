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

namespace mpc_new{

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
        float speed_target;
        float speed_actual;
        float motor_torque_target;
        float steering_angle_target;
        float brake_pressure_target;
    };

    class new_MpcSolver{
        public:
            new_MpcSolver();
            new_MpcSolver(double F_init, int horizonLength, double ds, double dt, double vel_max, double maxF, double minF, std::string mission, int laps);
            ~new_MpcSolver();

            constexpr static int X_SIZE = 8; //X = [x, y, phi, vx, vy, w, F, delta]
            constexpr static int U_SIZE = 2; //Y = [dF, ddelta]
            constexpr static int Z_SIZE = X_SIZE + U_SIZE; //Z = [x, y, phi, vx, vy, w, F, delta, dF, ddelta]
            double X[X_SIZE];
            double U[U_SIZE];
            int horizonLength;
            double dt;
            int points = 0;
            double vel_max;
            double s_interval_;
            output_data output_struct;
            double F_max;
            double F_min;
            int total_laps;
            PointsData whole_track;
            std::string mission;
            int global_counter = 0;
            bool is_out_of_map = false;

            NormalForces getFz(const double X[X_SIZE]);
            MuConstraints getEllipseParams(const double &Fz);
            SlipAngles getSlipAngles(const double X[X_SIZE]);
            double getFy(double Fz, double sa);
            double convertForceToPressure(float Frx_);
            void getF(double X[X_SIZE],double U[U_SIZE], double (&kappa)[X_SIZE]);
            void Integrator();
            bool check_reliability();

            std::vector<double> velocity_profile(const std::vector<double> x, const std::vector<double> y, std::vector<double> curv);
            void build(const double X_data[X_SIZE]);
            void choose(int t);
        private:
            double start_point[2];//first element is the x and second is y
            double max_velocity_h = 0; 
            double X2[X_SIZE];
            double X3[X_SIZE];
            double X4[X_SIZE];
            double k1[X_SIZE];
            double k2[X_SIZE];
            double k3[X_SIZE];
            double k4[X_SIZE];
            double wb = 1.590;
            double wd_front = 0.467;
            double l_f = wb*(1-wd_front);
            double l_r = wb*wd_front;
            double CdA = 2.0; 
            double ClA = 7.0; 
            double p_air = 1.225;
            double h_cog = 0.27;
            double gr = 3.9;
            double Rw = 0.2;
            double m = 190.0; 
            double g = 9.81;
            double Iz = 110.0;
            double ts = 0.025;
            double fr_par = 0.03;
            double wing = 0.874;
            int N_front = 4;
            int N_rear = 2;
            double d_piston = 0.025;
            double R_disk_f = 0.079;
            double R_disk_r = 0.0735;
            double mi_disk = 0.6;
            double umin=4.0;
            double umax=7.0;
            double eff = 0.85;
            int exitflag;
            bool finish = false;
            FORCESNLPsolver_params params;
            FORCESNLPsolver_info info;
            FORCESNLPsolver_output output;
            FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
            FORCESNLPsolver_mem *mem;
    };
}