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

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <unistd.h>
#include <fstream>
#include <string>
#include <array>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>
#include <iostream>
#include <string>
#include <filesystem>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/can_control_command.hpp"
#include "custom_msgs/msg/mpc_to_can.hpp"
#include "arc_length_spline.h"
#include "read_track.h"
#include "Eigen/Dense"


namespace mpc_cpp {
using namespace path_planning;
// needed for reading data only
const int LOOKAHEAD = 40;
std::vector<double> X_spl(LOOKAHEAD), Y_spl(LOOKAHEAD), tang_spl(LOOKAHEAD), curv_spl(LOOKAHEAD);
FORCESNLPsolver_params params;
FORCESNLPsolver_info info;
FORCESNLPsolver_output output;
FORCESNLPsolver_mem *mem;
FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
PointsData params_array;
const int X_SIZE = 9;
const int U_SIZE = 3;
const int Z_SIZE = X_SIZE + U_SIZE;
const int SIM = 1000;
const int SPLINE_RES = 2000;
//set physical constants
const double l_f = 0.9141;
const double l_r = 0.7359;
const double CdA = 1.8; // for drag
const double ClA = 5.47; // for downforce
const double p_air = 1.225;
const double gr = 3.9;
const double sr = 3.17;
const double Rw = 0.2;
const double m = 190.0; // mass of the car
const double g = 9.81;
const double Iz = 110.0;
const double ts = 0.05;

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
double X[X_SIZE];
double X2[X_SIZE];
double X3[X_SIZE];
double X4[X_SIZE];
double U[U_SIZE];
double k1[X_SIZE];
double k2[X_SIZE];
double k3[X_SIZE];
double k4[X_SIZE];
clock_t st_int, end_int, st_params, end_params;

struct node_out {
    double motor_torque;
    double brake_torque;
    double steering_angle;
};


double custom_max(double aa, double bb){
    if(aa<bb){
        return bb;
    }
    else {
        return aa;
    }
}

double custom_min(double aa, double bb){
    if(aa<bb){
        return aa;
    }
    else {
        return bb;
    }
}

void getF(double X[X_SIZE],double U[U_SIZE], double (&kappa)[X_SIZE])  {
    const double temp=(X[3]-umin)/(umax-umin);
    double double_1 = custom_max(temp,0);
    double l_a = custom_min(double_1,1);

    double saf=std::atan((X[4]+l_f*X[5])/(X[3]+1e-3)) - X[7];
    double sar=std::atan((X[4]-l_r*X[5])/(X[3]+1e-3));
    double Ffy = C_tire*D*std::sin(C*std::atan(B*saf));
    double Fry = C_tire*D*std::sin(C*std::atan(B*sar));
    double Frz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2));
    double Ffz = (l_r/(l_r+l_f))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2));
    double Fdrag = 0.5*CdA*p_air*std::pow(X[3],2) + 0.03*(Frz+Ffz);
    double beta = std::atan(l_r/(l_f + l_r) * std::tan(X[7]));

    kappa[0] = (l_a)*(X[3]*std::cos(X[2]) - X[4]*std::sin(X[2])) + (1-l_a)*(X[3]*std::cos(X[2] + beta));  
    kappa[1] = (l_a)*(X[3]*std::sin(X[2]) + X[4]*std::cos(X[2])) + (1-l_a)*(X[3]*std::sin(X[2] + beta));
    kappa[2] = X[5];
    kappa[3] = (1-l_a)*((X[6] - Fdrag)/ m) + (l_a)*((X[6] - Fdrag + Ffy*std::sin(X[7]) + m*X[4]*X[5])/ m); //vxdot
    kappa[4] = (1-l_a)*((l_r/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*(((-X[3]*X[5]) + (Fry + Ffy*std::cos(X[7])))/(1.0*m));
    kappa[5] = (1-l_a)*((1/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*((Ffy*l_f*std::cos(X[7]) - Fry*l_r)/(1.0*Iz));
    kappa[6] = U[0];
    kappa[7] = U[1];
    kappa[8] = U[2]; 
}

void Integrator() {
    st_int=clock();
    getF(X,U,k1);
    for (int i = 0; i<X_SIZE;++i) {
        X2[i] = X[i] +(ts/2)*k1[i];
    }
        getF(X2,U,k2);
        for (int i = 0; i<X_SIZE;++i) {
            X3[i] = X[i] +(ts/2)*k2[i];
        }
        getF(X3,U,k3);
        for (int i = 0; i<X_SIZE;++i) {
            X4[i] = X[i] +ts*k3[i];
        }
        getF(X4,U,k4);
        for (int i = 0; i<X_SIZE;++i) {
            double step_all = ts*((k1[i]/6) +(k2[i]/3) +(k3[i]/3) + (k4[i]/6));
            // std::cout << "step all is: " << step_all << std::endl;
            X[i] = X[i] + step_all;
        }
        end_int = clock();
        double time_taken_int = double(end_int - st_int) / double(CLOCKS_PER_SEC);
        std::cout << "Time for integration is: " << std::fixed << time_taken_int*1000 << std::setprecision(9); 
        std::cout << " ms " << std::endl;
} 

// void Initialize_all();
void Initialize_all_local(){
    for (int i = 0; i < Z_SIZE*LOOKAHEAD; i++) {
        params.x0[i] = 0.0;
        // std::cout << "i is: " << i << std::endl;
    }
    //X_array is: X  Y  phi vx vy  r F delta index
    const double xinit_temp[9] = {params_array(0,0),params_array(0,1),params_array(0,2),0.0,0.0,0.0,3560/10,0.0,0.0};
    // const double xinit_temp[9] = {0.0, 0.0,0.0,0.0,0.0,0.0,3560/10,0.0,0.0};
    for (int i = 0; i < X_SIZE; ++i) {
        X[i] = xinit_temp[i];
    }
    mem = FORCESNLPsolver_internal_mem(0);
    // node_out node_out;
}

void writeParamsLocal() {
    for (int i = 0; i < params_array.rows(); i++) {
        std:: cout << "params row" << i+1 << " is: " << params_array(i,0) << " " << params_array(i,1) << " " << params_array(i,2) << std::endl; 
        X_spl[i]=params_array(i, 0);
        Y_spl[i]=params_array(i, 1);
        tang_spl[i]=params_array(i, 2);
        curv_spl[i]=params_array(i, 3);
    }
    double X_loc = X_spl[0];
    double Y_loc = Y_spl[0];
    double phi_loc = tang_spl[0];
    X[0] = X_loc; //slam data update
    X[1] = Y_loc;
    X[2] = phi_loc;
    std::cout << "I'm at: " << X_loc << " " << Y_loc << " " << phi_loc << std::endl; 
    std::vector<double> ds_path;
    for(int i = 1; i<LOOKAHEAD-1; ++i) { //second to second to last for ds comparison
        ds_path.push_back(std::abs(std::sin(tang_spl[i])*(X_loc-X_spl[i]) - std::cos(tang_spl[i])*(Y_loc-Y_spl[i])));
        // ds_path.push_back(std::sqrt(std::pow((params.xinit[0] - X_spl[i]),2.0) + std::pow((params.xinit[1] - Y_spl[i]),2.0)));
    }
    int closest_index = std::distance(ds_path.begin(), std::min_element(ds_path.begin(), ds_path.end()));
    std::cout << "closest point is at index: " << closest_index+2 << std::endl;
    std::cout << "vertical distance is: " << ds_path[closest_index] << std::endl;
    for(int k = 0; k < 4*LOOKAHEAD; ++k) { 
        int mod_ = k%4;
        if (mod_ == 0) {
            params.all_parameters[k] = params_array(int(k/4),0); 
            // std::cout << "added X at kappa  " << k << std::endl;
        }
        else if(mod_ == 1) {
            params.all_parameters[k] = params_array(int((k-1)/4),1);
            // std::cout << "added Y at kappa " << k << std::endl;
        }
        else if(mod_ == 2) {
            params.all_parameters[k] = params_array(int((k-2)/4),2);
            // std::cout << "added tang at kappa " << k << std::endl;
        }
        else {
            params.all_parameters[k] = params_array(int((k-3)/4),3);
            // std::cout << "added curv at kappa " << k << std::endl;
        }
    }
    std::cout << "finished writing of all params" << std::endl;
}

} //end namespace