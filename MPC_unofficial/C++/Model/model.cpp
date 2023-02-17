// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "model.h"
namespace mpcc{
Model::Model()
:Ts_(1.0)
{
    std::cout << "default constructor of model, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{
}

ForceParameters Model::getForceParamsY(const double Fz) const { //checked
    const double C_tire=0.66;
    const double P [] = {1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04};
    const double ex=0.00001;

    const double C=P[0];
    const double D=P[1]*std::pow(Fz,2)+P[2]*Fz;
    const double BCD = P[3]*std::sin(P[4]*std::atan(P[5]*Fz));
    const double B = (BCD)/(C*D +ex);
    return {-B,C,C_tire,D};
}

Torques Model::getTorques(const State &x) const { //checked
    double T_G_temp = 0;  
    double T_Bf_temp = 0;
    double T_Br_temp = 0;	
	if(x.D>0 || x.D == 0){
		T_G_temp = param_.T_M_max*param_.gr*x.D;  
	}
	if(x.D < 0){
		T_Bf_temp=param_.Rdf*param_.mi_disk*param_.A_cal_f*param_.PMC_max_f*std::abs(x.D);
		T_Br_temp=param_.Rdr*param_.mi_disk*param_.A_cal_r*param_.PMC_max_r*std::abs(x.D);
	}
    return {T_G_temp,T_Bf_temp,T_Br_temp};
}

double Model::getSlipAngleFront(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy+x.r*param_.lf,x.vx) + x.delta;
}

double Model::getSlipAngleRear(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy-x.r*param_.lr,x.vx);
}

TireForces Model::getForceFront(const State &x) const
{
    NormalForces F_z = getForceNormal(x);
    ForceParameters par_y = getForceParamsY(F_z.F_N_front);
    Torques T = getTorques(x); 
    const double alpha_f = getSlipAngleFront(x);
    const double F_y = par_y.D*par_y.C_tire * std::sin(par_y.C * std::atan(par_y.B * alpha_f ));
    const double F_x = (-T.T_Bf)/(param_.Rf)  - param_.Cr0*F_z.F_N_front ;
    return {F_y,F_x};
}

TireForces Model::getForceRear(const State &x) const
{
    const double alpha_r = getSlipAngleRear(x);
    NormalForces F_z = getForceNormal(x);
    ForceParameters par_y = getForceParamsY(F_z.F_N_rear);
    Torques T = getTorques(x); 
    const double F_y = par_y.D*par_y.C_tire * std::sin(par_y.C * std::atan(par_y.B * alpha_r ));
    const double F_x = ((T.T_G-T.T_Br)/(param_.Rr))  - param_.Cr0*F_z.F_N_rear;// - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);
    // std::cout << "params for fy are:" << alpha_r << " " << par_y.B << " " << par_y.C << " " << par_y.C_tire*par_y.D << std::endl; 
    return {F_y,F_x};
}

double Model::getForceFriction(const State &x) const
{
    const double temp= -0.5*param_.CdA*param_.pair*std::pow(x.vx,2); 
    return temp;
}


NormalForces Model::getForceNormal(const State &x) const
{
    // at this point aero forces could be modeled
    const double f_n_front = param_.lr/(param_.lf + param_.lr)*param_.m*param_.g + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    const double f_n_rear = param_.lf/(param_.lf + param_.lr)*param_.m*param_.g +0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    return {f_n_front,f_n_rear}; 
}

TireForcesDerivatives Model::getForceFrontDerivatives(const State &x) const
{
    const double alpha_f = getSlipAngleFront(x);
    NormalForces F_z = getForceNormal(x);
    ForceParameters par_y = getForceParamsY(F_z.F_N_front);
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;

    // F_fx
    const double dF_x_vx    = (- 0.5*param_.Cr0*param_.pair*param_.ClA*x.vx); //-drag-trivi
    const double dF_x_vy    = 0.0;
    const double dF_x_r     = 0.0;
    const double dF_x_D     = (-param_.Rdf*param_.mi_disk*param_.A_cal_f*param_.PMC_max_f);
    const double dF_x_delta = 0.0;
    // F_fy
    const double dF_y_vx    = (par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_f)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_f,2))*((param_.lf*r + vy)
                                            /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_vy    = (par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_f)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_f,2))
                                            *(-vx/(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_r     =  (par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_f)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_f,2))*((-param_.lf*vx)
                                            /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_D     =  0.0;
    const double dF_y_delta = (par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_f)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_f,2));

    return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
}

TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const
{
    const double alpha_r = getSlipAngleRear(x);
    NormalForces F_z = getForceNormal(x);
    ForceParameters par_y = getForceParamsY(F_z.F_N_rear);
    Torques T = getTorques(x); 

    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D  = x.D;

    //F_rx
    const double dF_x_vx    = (- 0.5*param_.Cr0*param_.pair*param_.ClA*x.vx); //-trivi
    const double dF_x_vy    = 0.0;
    const double dF_x_r     = 0.0;
    const double dF_x_D     = (param_.T_M_max*param_.gr - param_.Rdr*param_.mi_disk*param_.A_cal_r*param_.PMC_max_r );
    const double dF_x_delta = 0.0;
    // F_ry
    const double dF_y_vx    = ((par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_r)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_r,2)))*(-(param_.lr*r - vy)
                                            /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_vy    = ((par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_r)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_r,2)))
                                            *((-vx)/(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_r     = ((par_y.B*par_y.C*par_y.D*par_y.C_tire*std::cos(par_y.C*std::atan(par_y.B*alpha_r)))
                                            /(1.+std::pow(par_y.B,2)*std::pow(alpha_r,2)))*((param_.lr*vx)
                                            /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_D     = 0.0;
    const double dF_y_delta = 0.0;

    return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
}

FrictionForceDerivatives Model::getForceFrictionDerivatives(const State &x) const
{
    // return {-param_.Cr0*param_.pair*param_.ClA*x.vx-param_.CdA*param_.pair*x.vx,0.0,0.0,0.0,0.0};
    return {-param_.CdA*param_.pair*x.vx,0.0,0.0,0.0,0.0};
}

// Binary function,
bool cmp(int x, int y) {
    return abs(x) < abs(y);
}

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

double Model::getLambda(const State &x) const{
    const double temp=(x.vx-3)/(18-3);
    double double_1 = custom_max(temp,0);
    double lambda = custom_min(double_1,1);
    // lambda=0;
    return lambda;
}

StateVector Model::getF(const State &x,const Input &u) const
{
    const double phi = x.phi;
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D = x.D;
    const double delta = x.delta;
    const double vs = x.vs;

    const double dD = u.dD;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

    const TireForces tire_forces_front = getForceFront(x);
    const TireForces tire_forces_rear  = getForceRear(x);
    const double friction_force = getForceFriction(x);
    double lambda = getLambda(x);

    StateVector f;
    f(0) = vx*std::cos(phi) - vy*std::sin(phi);
    f(1) = vy*std::cos(phi) + vx*std::sin(phi);
    f(2) = r;
    f(3) = (1.0/param_.m)* (lambda*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*std::sin(delta) + tire_forces_front.F_x*std::cos(delta) + param_.m*vy*r) + (1-lambda)*(tire_forces_rear.F_x+ param_.m*vy*r));
    f(4) = (1.0/param_.m)*(lambda*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) + tire_forces_front.F_x*std::sin(delta) - param_.m*vx*r)) + (1-lambda)*((param_.lf/(param_.lr+param_.lf))*(delta*tire_forces_rear.F_x + dDelta*vx));
    f(5) = (1.0/param_.Iz)*(lambda*(tire_forces_front.F_y*param_.lf*std::cos(delta) - tire_forces_rear.F_y*param_.lr + tire_forces_front.F_x*param_.lf*std::sin(delta))) + (1-lambda)*((1.0/(param_.lr+param_.lf))*(delta*tire_forces_rear.F_x + dDelta*vx));
    f(6) = vs;
    f(7) = dD;
    f(8) = dDelta;
    f(9) = dVs;

    // std::cout << "ax is" << f(3) << std::endl; 

    return f;
}

LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    // compute jacobian of the model
    // state values
    const double phi = x.phi;
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D = x.D;
    const double delta = x.delta;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

    const TireForces F_front = getForceFront(x);
    const TireForces F_rear = getForceRear(x);
    double lambda = getLambda(x);

//    TireForces F_rear  = getForceRear(x);

    const TireForcesDerivatives dF_front = getForceFrontDerivatives(x);
    const TireForcesDerivatives dF_rear  = getForceRearDerivatives(x);
    const FrictionForceDerivatives dF_fric = getForceFrictionDerivatives(x);

    // Derivatives of function
    // f1 = v_x*std::cos(phi) - v_y*std::sin(phi)
    const double df1_dphi = -vx*std::sin(phi) - vy*std::cos(phi);
    const double df1_dvx  = std::cos(phi);
    const double df1_dvy  = -std::sin(phi);

    // f2 = v_y*std::cos(phi) + v_x*std::sin(phi);
    const double df2_dphi = -vy*std::sin(phi) + vx*std::cos(phi);
    const double df2_dvx  = std::sin(phi);
    const double df2_dvy  = std::cos(phi);

    // f3 = r;
    const double df3_dr = 1.0;

    // f4 = 1/param_.m*(F_rx + F_fric - F_fy*std::sin(delta) + F_fx*std::cos(delta) param_.m*v_y*r);
    // f4 = 1/param_.m*(F_rx + F_fric - F_fy*std::sin(delta) + F_fx*std::cos(delta) param_.m*v_y*r);
    //(1-lambda)*(tire_forces_rear.F_x)
    //f(8) -> ddelta
    //f(3) -> vx
    const double df4_dvx     = (1.0/param_.m)*(lambda*(dF_rear.dF_x_vx + dF_fric.dF_f_vx - dF_front.dF_y_vx*std::sin(delta) + dF_front.dF_x_vx*std::cos(delta)) + (1-lambda)*(dF_rear.dF_x_vx));
    const double df4_dvy     = (1.0/param_.m)*(lambda*(- dF_front.dF_y_vy*std::sin(delta)    + param_.m*r) + (1-lambda)*(param_.m*r));
    const double df4_dr      = (1.0/param_.m)*(lambda*(- dF_front.dF_y_r*std::sin(delta)     + param_.m*vy)+ (1-lambda)*(param_.m*vy));
    const double df4_dD      = (1.0/param_.m)* (lambda*(dF_rear.dF_x_D + dF_front.dF_x_D*std::cos(delta)) + (1-lambda)*(dF_rear.dF_x_D));
    const double df4_ddelta  = (1.0/param_.m)*((lambda*(-dF_front.dF_y_delta*std::sin(delta) - F_front.F_y*std::cos(delta) - F_front.F_x*std::sin(delta))) + 0);

    // f5 = 1/param_.m*(F_ry + F_fy*std::cos(delta) + F_fx*std::sin(delta)- param_.m*v_x*r);
    const double df5_dvx     = (1.0/param_.m)*(lambda*(dF_rear.dF_y_vx  + dF_front.dF_y_vx*std::cos(delta)  + dF_front.dF_x_vx*std::sin(delta)  - param_.m*r)) + (1-lambda)*(param_.lr/(param_.lr+param_.lf))*(f(8)+delta*dF_rear.dF_x_vx);
    const double df5_dvy     = (1.0/param_.m)*(lambda*(dF_rear.dF_y_vy  + dF_front.dF_y_vy*std::cos(delta)));
    const double df5_dr      = (1.0/param_.m)*(lambda*(dF_rear.dF_y_r   + dF_front.dF_y_r*std::cos(delta) - param_.m*vx));
    const double df5_dD =      (1.0/param_.m)*(lambda*(dF_front.dF_x_D*std::sin(delta)))+(1-lambda)*(param_.lr/(param_.lr+param_.lf))*(delta*dF_rear.dF_x_D);
    const double df5_ddelta  = (1.0/param_.m)*(lambda*( dF_front.dF_y_delta*std::cos(delta) - F_front.F_y*std::sin(delta) + F_front.F_x*std::cos(delta))) +(1-lambda)*(param_.lr/(param_.lr+param_.lf))*(F_rear.F_x);

    // f6 = 1/param_.Iz*(F_fy*l_f*std::cos(delta)- F_ry*l_r +F_fx*lf*sin(delta))
    const double df6_dvx     = (1.0/param_.Iz)*(lambda*(dF_front.dF_y_vx*param_.lf*std::cos(delta)    - dF_rear.dF_y_vx*param_.lr + dF_front.dF_x_vx*param_.lf*std::sin(delta))) +(1-lambda)*(1/(param_.lr+param_.lf))*(f(8)+(delta*dF_rear.dF_x_vx));
    const double df6_dvy     = (1.0/param_.Iz)*(lambda*(dF_front.dF_y_vy*param_.lf*std::cos(delta)    - dF_rear.dF_y_vy*param_.lr));
    const double df6_dr      = (1.0/param_.Iz)*(lambda*(dF_front.dF_y_r*param_.lf*std::cos(delta)     - dF_rear.dF_y_r*param_.lr));
    const double df6_dD = (1.0/param_.Iz)*(lambda*(dF_front.dF_x_D*param_.lf*std::sin(delta))) + (1-lambda)*(1/(param_.lr+param_.lf))*(delta*dF_rear.dF_x_D);
    const double df6_ddelta  = (1.0/param_.Iz)*(lambda*(dF_front.dF_y_delta*param_.lf*std::cos(delta) - F_front.F_y*param_.lf*std::sin(delta)+F_front.F_x*param_.lf*std::cos(delta))) + (1-lambda)*(1/(param_.lr+param_.lf))*(F_rear.F_x);

    // Jacobians
    // Matrix A
    // Column 1
    // all zero
    // Column 2
    // all zero
    // Column 3
    A_c(0,2) = df1_dphi;
    A_c(1,2) = df2_dphi;
    // Column 4
    A_c(0,3) = df1_dvx;
    A_c(1,3) = df2_dvx;
    A_c(3,3) = df4_dvx;
    A_c(4,3) = df5_dvx;
    A_c(5,3) = df6_dvx;
    // Column 5
    A_c(0,4) = df1_dvy;
    A_c(1,4) = df2_dvy;
    A_c(3,4) = df4_dvy;
    A_c(4,4) = df5_dvy;
    A_c(5,4) = df6_dvy;
    // Column 6
    A_c(2,5) = df3_dr;
    A_c(3,5) = df4_dr;
    A_c(4,5) = df5_dr;
    A_c(5,5) = df6_dr;
    // Column 7
    // all zero
    // Column 8
    A_c(3,7) = df4_dD;
    A_c(4,7) = df5_dD;
    A_c(5,7) = df6_dD;
    // Column 9
    A_c(3,8) = df4_ddelta;
    A_c(4,8) = df5_ddelta;
    A_c(5,8) = df6_ddelta;
    // Column 10
    A_c(6,9) = 1.0;

    // Matrix B
    // Column 1
    B_c(7,0) = 1.0;
    // Column 2
    B_c(8,1) = 1.0;
    // Column 3
    B_c(9,2) = 1.0;

    //zero order term
    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);
    //std::cout << g_c << std::endl;
    //std::cout << " " << std::endl;
    //std::cout << A_c << std::endl;
    return {A_c,B_c,g_c};
}

// Mixed RK4 - EXPM method
LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c, const State &x, const Input &u,const State &x_next) const
{
    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
    Eigen::Matrix<double,NX+NU,NX+NU> temp = Eigen::Matrix<double,NX+NU,NX+NU>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX,NX>(0,0) = lin_model_c.A;
    temp.block<NX,NU>(0,NX) = lin_model_c.B;
    temp = temp*Ts_;

    // take the matrix exponential of temp
    const Eigen::Matrix<double,NX+NU,NX+NU> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k
    //temp_res = [Ad,Bd;zeros]
    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);

    // TODO: use correct RK4 instead of inline RK4
    const StateVector x_vec = stateToVector(x);

    const StateVector k1 = getF(vectorToState(x_vec),u);
    const StateVector k2 = getF(vectorToState(x_vec+Ts_/2.*k1),u);
    const StateVector k3 = getF(vectorToState(x_vec+Ts_/2.*k2),u);
    const StateVector k4 = getF(vectorToState(x_vec+Ts_*k3),u);
    // combining to give output
    const StateVector x_RK = x_vec + Ts_*(k1/6.+k2/3.+k3/3.+k4/6.);

    const g_MPC g_d =  -stateToVector(x_next) + x_RK;

    // return {A_d,B_d,g_d};
    return {A_d,B_d,g_d};
}

// // EF Method (not suited for used input lifting technique)
// LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c, const State &x, const Input &u,const State &x_next) const
// {
//     // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
//     Eigen::Matrix<double,NX+NU,NX+NU> temp = Eigen::Matrix<double,NX+NU,NX+NU>::Zero();
//     // building matrix necessary for expm
//     // temp = Ts*[A,B,g;zeros]
//     temp.block<NX,NX>(0,0) = lin_model_c.A;
//     temp.block<NX,NU>(0,NX) = lin_model_c.B;
//     temp = temp*Ts_;

//     Eigen::Matrix<double,NX+NU,NX+NU> eye;
//     eye.setIdentity();
//     // take the matrix exponential of temp
//     const Eigen::Matrix<double,NX+NU,NX+NU> temp_res = eye + temp;
//     // extract dynamics out of big matrix
//     // x_{k+1} = Ad x_k + Bd u_k
//     //temp_res = [Ad,Bd;zeros]
//     const A_MPC A_d = temp_res.block<NX,NX>(0,0);
//     const B_MPC B_d = temp_res.block<NX,NU>(0,NX);

//     const StateVector x_vec = stateToVector(x);

//     const StateVector f = getF(vectorToState(x_vec),u);
//     // combining to give output
//     const StateVector x_EF = x_vec + Ts_*f;

//     const g_MPC g_d =  -stateToVector(x_next) + x_EF;
//     // return {A_d,B_d,g_d};
//     return {A_d,B_d,g_d};
// }

LinModelMatrix Model::getLinModel(const State &x, const Input &u, const State &x_next) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c,x,u,x_next);
}
}
