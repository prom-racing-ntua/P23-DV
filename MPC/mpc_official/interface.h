extern "C" {
#include <stdio.h>
#include "FORCESNLPsolver/include/FORCESNLPsolver.h"
#include "FORCESNLPsolver/include/FORCESNLPsolver_memory.h"

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
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>

namespace mpc_cpp {
// needed for reading data only
std::string filename = "data/cubic_spline_data.txt"; // replace with the name of your text file
std::ifstream infile(filename);
std::vector<double> X_spl, Y_spl, tang_spl, curv_spl;
std::string line;
FORCESNLPsolver_params params;
FORCESNLPsolver_info info;
FORCESNLPsolver_output output;
FORCESNLPsolver_mem *mem;
FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
const int X_SIZE = 9;
const int LOOKAHEAD = 40;
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

struct node_out {
    double motor_torque;
    double brake_torque;
    double steering_angle;
};


double custom_max(double a, double b){
    if(a<b){
        return b;
    }
    else {
        return a;
    }
}

double custom_min(double a, double b){
    if(a<b){
        return a;
    }
    else {
        return b;
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
} 

void Initialize_all(){
    for (int i = 0; i < Z_SIZE*LOOKAHEAD; i++) {
        params.x0[i] = 0.0;
    }
    double xinit_temp[9] = {X_spl[0], Y_spl[0],tang_spl[0],0.0,0.0,0.0,3560/10,0.0,0.0}; //X_array is: X  Y  phi vx vy  r F delta index
    for (int i = 0; i < X_SIZE; ++i) {
        X[i] = xinit_temp[i];
    }
    mem = FORCESNLPsolver_internal_mem(0);
    node_out node_out;
}

void readData() {
    while (getline(infile, line)) {
        std::istringstream iss(line);
        double value;
        if (iss >> value) {
            X_spl.push_back(value);
        }
        if (iss >> value) {
            Y_spl.push_back(value);
        }
        if (iss >> value) {
            tang_spl.push_back(value);
        }
        if (iss >> value) {
            curv_spl.push_back(value);
        }
    }
}

void writeParams() {
    double X_spline_temp = X_spl[int(params.xinit[8])];
    double Y_spline_temp = Y_spl[int(params.xinit[8])];
    std::cout << "splines are at: " << X_spline_temp << " " << Y_spline_temp << std::endl;
    std::cout << "I'm at: " << params.xinit[0] << " " << params.xinit[1] << std::endl;
    double phi_spline_temp = tang_spl[int(params.xinit[8])];
    double e_c =  std::sin(phi_spline_temp)*(params.xinit[0]-X_spline_temp) - std::cos(phi_spline_temp)*(params.xinit[1]-Y_spline_temp);
    double e_l =  -std::cos(phi_spline_temp)*(params.xinit[0]-X_spline_temp) - std::sin(phi_spline_temp)*(params.xinit[1]-Y_spline_temp);
    double e_eucl = std::sqrt(std::pow((params.xinit[0] - X_spline_temp),2.0) + std::pow((params.xinit[1] - Y_spline_temp),2.0));
    std::cout << "distances are: " << std::abs(e_c) << " " << std::abs(e_l) << " " << std::abs(e_eucl) << std::endl;
    if(std::abs(e_c)>1.0) { //safety factor when going off-path
        std::vector<double> ds_path;
        for(int i = 0; i<SPLINE_RES; ++i) {
            // ds_path.push_back(std::abs(std::sin(tang_spl[i])*(params.xinit[0]-X_spl[i]) - std::cos(tang_spl[i])*(params.xinit[1]-Y_spl[i])));
            ds_path.push_back(std::sqrt(std::pow((params.xinit[0] - X_spl[i]),2.0) + std::pow((params.xinit[1] - Y_spl[i]),2.0)));
        }
        int new_index = std::distance(ds_path.begin(), std::min_element(ds_path.begin(), ds_path.end()));
        params.xinit[8] = new_index;
        std::cout << "too far away but found new index " << ds_path.size() << " " << params.xinit[8] << std::endl;
    }
    std::cout << "params index is " << params.xinit[8] << std::endl;
    for(int k = 0; k < 4*LOOKAHEAD; ++k) { //set parameters
        int mod_ = k%4;
        // std::cout << "mod is: " << mod_ << std::endl;
        int ktemp1 = int(k/4);
        int ktemp2 = int((k-1)/4);
        int ktemp3 = int((k-2)/4);
        int ktemp4 = int((k-3)/4);
        if (mod_ == 0) {
            params.all_parameters[k] = X_spl[int(params.xinit[8])+ ktemp1];
            // std::cout << "added X at kappa  " << k << std::endl;
        }
        else if(mod_ == 1) {
            params.all_parameters[k] = Y_spl[int(params.xinit[8])+ ktemp2];
            // std::cout << "added Y at kappa " << k << std::endl;
        }
        else if(mod_ == 2) {
            params.all_parameters[k] = tang_spl[int(params.xinit[8])+ktemp3];
            // std::cout << "added tang at kappa " << k << std::endl;
        }
        else {
            params.all_parameters[k] = curv_spl[int(params.xinit[8])+ktemp4];
            // std::cout << "added curv at kappa " << k << std::endl;
        }
        }
}
} //end namespace