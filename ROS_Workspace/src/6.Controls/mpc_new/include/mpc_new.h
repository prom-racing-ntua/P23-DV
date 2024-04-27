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

    //Ellipses parameters
    struct MuConstraints {
    double mx_max;
    double my_max;
    };

    //z-Forces
    struct NormalForces {
        double Ffz;
        double Frz;
    };

    //Slip angles
    struct SlipAngles{
        double saf;
        double sar;
    };

    //Produced output to give instructions to the inverter
    struct output_data {
        float speed_target;
        float speed_actual;
        float motor_torque_target;
        float steering_angle_target;
        float brake_pressure_target;
    };

    class new_MpcSolver{
        public:
            //Constructors & Destructor
            new_MpcSolver();
            new_MpcSolver(double F_init, int horizonLength, double ds, double dt, double vel_max, double maxF, double minF, std::string mission,
                double time_delay, double T_max, double T_min, double angle_max, double angle_min, double wb, double wd_front, double CdA,
                double ClA, double p_air, double h_cog, double gr, double Rw, double m, double g, double Iz, int N_rear, double d_piston,
                double R_disk_f, double R_disk_r, double mi_disk, bool dynamic_ds);
            ~new_MpcSolver();

            constexpr static int X_SIZE = 8; //X = [x, y, phi, vx, vy, w, F, delta]
            constexpr static int U_SIZE = 2; //Y = [dF, ddelta]
            constexpr static int Z_SIZE = X_SIZE + U_SIZE; //Z = [x, y, phi, vx, vy, w, F, delta, dF, ddelta]
            double X[X_SIZE]; //State vector
            double U[U_SIZE]; //Input vector
            output_data output_struct;
            int total_laps; //The lap on which i am 
            PointsData whole_track; //The points (x, y, phi, curvature) pf the spline
            bool is_out_of_map = false; //Path planning boolean
            path_planning::ArcLengthSpline *spline;
            int global_counter = 0;
            double s_interval_; //ds on the spline
            std::string mission; //The event i am on

            //Extracting the z-Forces on the tires (VD)
            NormalForces getFz(const double X[X_SIZE]);

            //Extracting the ellipse parameters on the tires (VD)
            MuConstraints getEllipseParams(const double &Fz);

            //Extracting the slpi angles (VD)
            SlipAngles getSlipAngles(const double X[X_SIZE]);

            //Extracting the y-Forces on the tires (VD)
            double getFy(double Fz, double sa);

            //Converting the F i get from the model to pressure
            double convertForceToPressure(float Frx_);

            //Helpful function for the RK4 integration
            void getF(double X[X_SIZE],double U[U_SIZE], double (&kappa)[X_SIZE]);

            //RK4 integration
            void Integrator();

            //A small check on the ellipses
            bool check_reliability();

            //The velocity profile 
            std::vector<double> velocity_profile(const std::vector<double> x, const std::vector<double> y, const std::vector<double> curv);
            
            //The main function which combines the data(taking data from sensors, extracting the runtime parameters, calling the solver 
            //and producing the output)
            void build(const double X_data[X_SIZE]);

            //A function which helps me choose which output vector i want and assigns a helpful vector 
            void choose_and_assign(int t, int u);

            //The function which produces the output (on the output struct)
            void produce_output();

            //The function which builds dynamic ds (and gives runtime parameters with dynamic distances)
            std::vector<double> dynamic(std::vector<double> &x_data, std::vector<double> &y_data, std::vector<double> &phi_data, std::vector<double> &v_data, double current_s);
        private:
            int horizonLength; //Horizon (N)
            double dt; //Frequency (0.025)
            int points = 0; //For the sliding window to find the closest vertex
            double vel_max; //Maximum velocity allowed
            double F_max; //Maximum force allowed
            double F_min; //Minimum force allowed
            bool dynamic_ds; //Allowing dynamic ds
            double delay; //The delay on the system
            double T_max; //Maximum torque
            double T_min; //Minimum torque
            double angle_max; //Maximum steering angle
            double angle_min; //Minimum steering angle
            //For the RK4 integration
            double X2[X_SIZE];
            double X3[X_SIZE];
            double X4[X_SIZE];
            double k1[X_SIZE];
            double k2[X_SIZE];
            double k3[X_SIZE];
            double k4[X_SIZE];
            //Model parameters
            double wb;
            double wd_front;
            double l_f; 
            double l_r; 
            double CdA; 
            double ClA; 
            double p_air;
            double h_cog;
            double gr;
            double Rw;
            double m; 
            double g;
            double Iz;
            int N_rear;
            double d_piston;
            double R_disk_f;
            double R_disk_r;
            double mi_disk;
            //For the blended model
            double umin=4.0;
            double umax=7.0;
            double eff = 0.85;
            double safety_factor = 0.8;
            int exitflag = 1; //Solver's flag
            bool finish = false; //Check if i must stop or not
            //Variables that will help me handle the solver
            FORCESNLPsolver_params params;
            FORCESNLPsolver_info info;
            FORCESNLPsolver_output output;
            FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
            FORCESNLPsolver_mem *mem;
            int counting_errors = 0; //Helpful variable for when the solver does not solve the problem
            std::vector<std::vector<double>> prevx0; //Helpful vector to handle the solver's data
    };
}