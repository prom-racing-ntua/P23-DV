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

// double Model::getKappa(double Fz) const{ 
// 	const double C_tire=0.66;
// 	const double Fz0=1112.0554070627252;
// 	const double P [] = {1.35456598e+00, -2.21891927e+00, -1.36151651e-07,  1.26548452e-01, -5.02341855e-01, -2.38448523e+00, -5.61761294e-01,  4.27383067e+01,-9.12030469e+01,  9.96044571e-01};

// 	const double dfz=(Fz-Fz0)/Fz0;
// 	return C_tire*Fz*(P[7]+P[8]*dfz)*std::exp(P[9]*dfz);
// }


// double Model::getKappa2(double Fz) const{
//     const double C_tire=0.66;
//     const double Fz0=1112.0554070627252;
//     const double P [] = {40.42348858 -21.60666053  -1.00099207};
//     const double dfz=(Fz-Fz0)/Fz0;
//     return  C_tire *Fz0 *(P[0]+std::abs(P[1])*dfz)*std::exp(P[2]*dfz);
// }


Torques Model::getTorques(const State &x) const { //checked
	double T_G_temp=0;
	double T_Bf_temp=0;
	double T_Br_temp=0;
    double T_Bf_temp_new=0.2;
	if(x.D>0 || x.D == 0){
		T_G_temp = param_.T_M_max*param_.gr*x.D;  	
	}
	if(x.D < 0){
		T_Bf_temp=param_.Rdf*param_.mi_disk*param_.A_cal_f*param_.PMC_max_f*std::abs(x.D);
		T_Br_temp=param_.Rdr*param_.mi_disk*param_.A_cal_r*param_.PMC_max_r*std::abs(x.D);
	}
    // std::cout << "torques are: " << T_G_temp << " " << T_Bf_temp << " " << T_Br_temp << std::endl;
	return {T_G_temp,T_Bf_temp,T_Br_temp};
}

ForceParameters Model::getForceParamsX(const double Fz) const { //checked
    const double C_tire=0.66; 
    const double P [] = {1.48999993e+00, -1.27825382e-04,  2.34264116e+00,  1.83007708e-01, 6.57800078e+00,  1.41720038e-03};
    const double ex=0.00001;

    const double C=P[0];
    const double D=P[1]*std::pow(Fz,2)+P[2]*Fz;
    const double BCD = (P[3]*std::pow(Fz,2)+P[4]*Fz)/(std::exp(P[5]*Fz));
    const double B = (BCD)/(C*D +ex);
    
    return {B,C,C_tire,D};
}

ForceParameters Model::getForceParamsY(const double Fz) const { //checked
    const double C_tire=0.66;
    const double P [] = {1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04};
    const double ex=0.00001;

    const double C=P[0];
    const double D=P[1]*std::pow(Fz,2)+P[2]*Fz;
    const double BCD = P[3]*std::sin(P[4]*std::atan(P[5]*Fz));
    const double B = (BCD)/(C*D +ex);
    
    return {B,C,C_tire,D};
}


double Model::getSlipAngleFront(const State &x) const
{
    // compute slip angels given current state
    const double lf=getLengthFront(x);
    return -std::atan2(x.vy+x.r*lf,x.vx) + x.delta;
}   


SlipAngles Model::getSlipAngles(const State &x) const //checked
{
    // compute slip angels given current state
    const double sign=1;
    const double lf=getLengthFront(x);
    const double lr=getLengthRear(x);
    const double sa_f= sign*(std::atan2(x.vy+x.r*lf,x.vx) - x.delta);
    const double sa_r= sign*(std::atan2(x.vy-lr*x.r,x.vx));
    // std::cout << "slip angles are: " << sa_f << " " << sa_r << std::endl;
    return {sa_f,sa_r};
}

SlipRatios Model::getSlipRatios(const State &x) const //checked
{
    // compute slip angels given current state
    const double sign=1;
    const double lf=getLengthFront(x);
    const double lr=getLengthRear(x);
    const double num = x.rwf*param_.Rf - x.vx*std::cos(x.delta) - std::sin(x.delta)*(x.vy+lf*x.r);
    const double denum = x.vx*std::cos(x.delta) + std::sin(x.delta)*(x.vy+lf*x.r);
    const double sr_f = sign*(num/denum);
    const double sr_r = sign*((x.rwr*param_.Rr-x.vx)/(x.vx));
    // std::cout << "velocities2 for srs are: " << x.rwr*param_.Rr << " " << x.vx << std::endl;
    // std::cout << "slip ratios are: " << sr_f << " " << sr_r << std::endl;
    return {sr_f,sr_r};
}

SlipAngleDerivatives Model::getSlipAngleDerivatives(const State &x) const { //checked
    const double lf=getLengthFront(x);
    const double lr=getLengthRear(x);
    const double sign=1;


    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double rwr = x.rwr;
    const double rwf = x.rwf;
    const double delta= x.delta;
    const double D=x.D;

    //front
    const double denom = std::pow(vx,2)+ std::pow(vy+lf*r,2);
    const double dsaf_vx=sign*(-((vy+r*lf))/(denom));
    const double dsaf_vy=sign*(vx/denom);
    const double dsaf_r=sign*((vx*lf)/(denom));
    const double dsaf_delta=sign*(-1);

    //rear
    const double denom2 = std::pow(vx,2)+ std::pow(vy-lr*r,2);
    const double dsar_vx=sign*((lr*r-vy)/(denom2));
    const double dsar_vy=sign*(vx/denom2);
    const double dsar_r=sign*((-vx*lr)/(denom2));

    return {dsaf_vx,dsaf_vy,dsaf_r,dsaf_delta,dsar_vx,dsar_vy,dsar_r};
}

SlipRatioDerivatives Model::getSlipRatioDerivatives(const State &x) const{ //checked
    const double lf=getLengthFront(x);
    const double lr=getLengthRear(x);
    const double sign=1;

    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double rwr = x.rwr;
    const double rwf = x.rwf;
    const double delta= x.delta;
    const double D=x.D;

    const double denom=x.vx*std::cos(x.delta) +std::sin(x.delta)*(x.vy+lf*x.r);
    const double dsrf_vx=sign*((-rwf*param_.Rf*std::cos(delta))/(std::pow(denom,2)));
    const double dsrf_vy=sign*((-rwf*param_.Rf*std::sin(delta))/(std::pow(denom,2)));
    const double dsrf_r=sign*((-rwf*param_.Rf*lf*std::sin(delta))/(std::pow(denom,2)));
    const double dsrf_rwf=sign*(param_.Rf/denom);
    const double dsrf_delta=sign*((rwf*param_.Rf/std::pow(denom,2))*(vx*(std::sin(delta)) - (std::cos(delta))*(vy+lf*r)));

    const double dsrr_vx=sign*((-rwr*param_.Rr)/(std::pow(vx,2)));
    const double dsrr_rwr=sign*(param_.Rr/vx);

    return {dsrf_vx,dsrf_vy,dsrf_r,dsrf_rwf,dsrf_delta,dsrr_vx,dsrr_rwr};
}



double Model::getLengthRear(const State &x) const { //checked
    return param_.lol*param_.wd; // - param_.h_cog*a.ax;
}

double Model::getLengthFront(const State &x) const { //checked
    return param_.lol*(1-param_.wd); //+ param_.h_cog*a.ax;
}

TireForces Model::getForceFront(const State &x) const //checked
{   
    //const Acceleration a = getAcceleration(x);
    //const double F_z=param_.m*param_.g*(param_.wd - (a.ax*param_.h_cog/param_.lol)) + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    const double F_z=param_.m*param_.g*(param_.wd) + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    ForceParameters par_x = getForceParamsX(F_z);
    ForceParameters par_y = getForceParamsY(F_z);
    SlipAngles sa = getSlipAngles(x);
    SlipRatios sr = getSlipRatios(x);
    const double F_y = par_y.C_tire*par_y.D*(std::sin(par_y.C*std::atan(par_y.B*sa.sa_f)));
    const double F_x = par_x.C_tire*par_x.D*(std::sin(par_x.C*std::atan(par_x.B*sr.sr_f))) - 0.25*param_.CdA*param_.pair*::pow(x.vx,2.0);
    return {F_x,F_y,F_z};
}


TireForces Model::getForceRear(const State &x) const //checked
{
    //const Acceleration a = getAcceleration(x);
    const double F_z=param_.m*param_.g*(1-param_.wd) + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    //const double F_z=param_.m*param_.g*((1-param_.wd) + (a.ax*param_.h_cog/param_.lol)) + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    ForceParameters par_x = getForceParamsX(F_z);
    ForceParameters par_y = getForceParamsY(F_z);
    SlipAngles sa = getSlipAngles(x);
    SlipRatios sr = getSlipRatios(x);

    const double F_y = par_y.C_tire*par_y.D*(std::sin(par_y.C*std::atan(par_y.B*sa.sa_r)));
    const double F_x = par_x.C_tire*par_x.D*(std::sin(par_x.C*std::atan(par_x.B*sr.sr_r))) - 0.25*param_.CdA*param_.pair*std::pow(x.vx,2.0);
    // std::cout << "force params rear are: "  << par_y.C_tire*par_y.D << " " << sa.sa_r << " " << par_y.C*std::atan(par_y.B*sa.sa_r) << " " << par_y.B*sa.sa_r << " " << std::endl;
    return {F_x,F_y,F_z};
}



NormalForces Model::getForceNormal(const State &x) const
{
    const double lr=getLengthRear(x);
    const double lf=getLengthFront(x);
    const double f_n_front = lr/(lf + lr)*param_.m*param_.g + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    const double f_n_rear = lf/(lf + lr)*param_.m*param_.g + 0.25*param_.pair*param_.ClA*std::pow(x.vx,2.0);
    return {f_n_front,f_n_rear};
}

TireForcesDerivatives Model::getForceFrontDerivatives(const State &x) const //checked
{
    const double lf = getLengthFront(x);
    const TireForces tire_forces = getForceFront(x); 
    const ForceParameters pars_x=getForceParamsX(tire_forces.F_z);
    const ForceParameters pars_y=getForceParamsY(tire_forces.F_z);
    const SlipRatios sr = getSlipRatios(x);
    const SlipAngles sa = getSlipAngles(x);
    const SlipRatioDerivatives dsr = getSlipRatioDerivatives(x);
    const SlipAngleDerivatives dsa = getSlipAngleDerivatives(x);
    //const KappaDerivatives dk_Front = getKappaDerivatives(Fz,dFz_vx,dFz_ax);

    const double Bx=pars_x.B;
    const double C_tire_x=pars_x.C_tire;
    const double Cx=pars_x.C;
    const double Dx=pars_x.D;

    const double By=pars_y.B;
    const double C_tire_y=pars_y.C_tire;
    const double Cy=pars_y.C;
    const double Dy=pars_y.D;

    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double rwr = x.rwr;
    const double rwf = x.rwf;
    const double delta= x.delta;
    const double D=x.D;

    const double srf=sr.sr_f;
    const double saf=sa.sa_f;

    // Ffx
    const double stath_x=(1/(1+std::pow(Bx*srf,2)))*Bx*Cx*C_tire_x*Dx*std::cos(Cx*std::atan(Bx*srf));
    const double dF_x_vx    = stath_x*dsr.dsrf_vx - 0.5*param_.CdA*vx;
    const double dF_x_vy    = stath_x*dsr.dsrf_vy;
    const double dF_x_r    = stath_x*dsr.dsrf_r;
    const double dF_x_rwf    = stath_x*dsr.dsrf_rwf;
    const double dF_x_rwr = 0;
    const double dF_x_delta = 0 ;
    // Ffy
    const double stath_y=(1/(1+std::pow(By*saf,2)))*By*Cy*C_tire_y*Dy*std::cos(Cy*std::atan2(By*saf,1));
    const double dF_y_vx    = stath_y*dsa.dsaf_vx;
    const double dF_y_vy    = stath_y*dsa.dsaf_vy;
    const double dF_y_r    = stath_y*dsa.dsaf_r;
    const double dF_y_rwf   = 0;
    const double dF_y_rwr = 0;
    const double dF_y_delta = stath_y*dsa.dsaf_delta; 

    //Ffz
    const double dF_z_vx = 0.5*param_.pair*param_.ClA*vx;

    return {dF_x_vx,dF_x_vy,dF_x_r,dF_x_rwf,dF_x_rwr,dF_x_delta,dF_y_vx,dF_y_vy,dF_y_r,dF_y_rwf,dF_y_rwr,dF_y_delta,dF_z_vx};
}

TorquesDerivatives Model::getTorquesDerivatives(const State &x) const { //checked
    const double dTG_D_temp = param_.gr*param_.T_M_max;
    const double dTBf_D_temp = param_.Rdf*param_.mi_disk*param_.A_cal_f*param_.PMC_max_f;
    const double dTBr_D_temp = param_.Rdr*param_.mi_disk*param_.A_cal_r*param_.PMC_max_r;
    return {dTG_D_temp,dTBf_D_temp,dTBr_D_temp};
}


TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const //checked
{
    
    const double lf = getLengthFront(x);
    const TireForces tire_forces = getForceRear(x); 
    const ForceParameters pars_x=getForceParamsX(tire_forces.F_z);
    const ForceParameters pars_y=getForceParamsY(tire_forces.F_z);
    const SlipRatios sr = getSlipRatios(x);
    const SlipAngles sa = getSlipAngles(x);
    const SlipRatioDerivatives dsr = getSlipRatioDerivatives(x);
    const SlipAngleDerivatives dsa = getSlipAngleDerivatives(x);
    //const KappaDerivatives dk_Front = getKappaDerivatives(Fz,dFz_vx,dFz_ax);

    const double Bx=pars_x.B;
    const double C_tire_x=pars_x.C_tire;
    const double Cx=pars_x.C;
    const double Dx=pars_x.D;

    const double By=pars_y.B;
    const double C_tire_y=pars_y.C_tire;
    const double Cy=pars_y.C;
    const double Dy=pars_y.D;

    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double rwr = x.rwr;
    const double rwf = x.rwf;
    const double delta= x.delta;
    const double D=x.D;

    const double srr=sr.sr_r;
    const double sar=sa.sa_r;

    // Frx
    const double stath_x=(1/(1+std::pow(Bx*srr,2)))*Bx*Cx*C_tire_x*Dx*std::cos(Cx*std::atan(Bx*srr));
    const double dF_x_vx    = stath_x*dsr.dsrr_vx - 0.5*param_.CdA*vx;
    const double dF_x_vy    = 0;
    const double dF_x_r    = 0;
    const double dF_x_rwf = 0;
    const double dF_x_rwr = stath_x*dsr.dsrr_rwr;
    const double dF_x_delta = 0;
    // Fry
    const double stath_y=(1/(1+std::pow(By*sar,2)))*By*Cy*C_tire_y*Dy*std::cos(Cy*std::atan2(By*sar,1));
    const double dF_y_vx    = stath_y*dsa.dsar_vx;
    const double dF_y_vy    = stath_y*dsa.dsar_vy;
    const double dF_y_r    = stath_y*dsa.dsar_r;
    const double dF_y_rwf    = 0;
    const double dF_y_rwr = 0;
    const double dF_y_delta = 0;

    //Ffz
    const double dF_z_vx = 0.5*param_.pair*param_.ClA*vx;

    return {dF_x_vx,dF_x_vy,dF_x_r,dF_x_rwf,dF_x_rwr,dF_x_delta,dF_y_vx,dF_y_vy,dF_y_r,dF_y_rwf,dF_y_rwr,dF_y_delta,dF_z_vx};
}


StateVector Model::getF(const State &x,const Input &u) const //checked
{
    const double phi = x.phi;
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double rwf=x.rwf;
    const double rwr=x.rwr;
    const double D = x.D;
    const double delta = x.delta;
    const double vs = x.vs;

    const double dD = u.dD;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

    const TireForces tire_forces_front = getForceFront(x);
    const TireForces tire_forces_rear  = getForceRear(x);
    const double lf=getLengthFront(x);
    const double lr=getLengthRear(x);
    const Torques torques_all = getTorques(x);
    SlipAngles sa = getSlipAngles(x);
    SlipRatios sr = getSlipRatios(x);
    // std::cout << "forces for vx are: " << tire_forces_rear.F_x << " " << tire_forces_front.F_y*std::sin(delta) << std::endl;


    StateVector f;
    f(0) = vx*std::cos(phi) - vy*std::sin(phi);
    f(1) = vy*std::cos(phi) + vx*std::sin(phi);
    f(2) = r;
    f(3) = (1.0/param_.m)*(tire_forces_rear.F_x - tire_forces_front.F_y*std::sin(delta) + tire_forces_front.F_x*std::cos(delta)) + vy*r;
    f(4) = (1.0/param_.m)*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) + tire_forces_front.F_x*std::sin(delta)) - vx*r;
    f(5) = (1.0/param_.Iz)*(tire_forces_front.F_y*lf*std::cos(delta) - tire_forces_rear.F_y*lr+ tire_forces_front.F_x*lf*std::sin(delta));
    f(6) = (1.0/param_.Iw)*(-tire_forces_front.F_x*param_.Rf - torques_all.T_Bf);
    f(7) = (1.0/param_.Iw)*(torques_all.T_G-torques_all.T_Br-param_.Rr*tire_forces_rear.F_x);
    f(8) = vs;
    f(9) = dD;
    f(10) = dDelta;
    f(11) = dVs;
    // std::cout << "ax_ol is: " << f(3) << std::endl;
    
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
    const double rwf=x.rwf;
    const double rwr=x.rwr;
    const double D = x.D;
    const double delta = x.delta;
    const double vs = x.vs;

    const double dD = u.dD;
    const double dDelta = u.dDelta;
    const double dVs = u.dVs;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);
    //std::cout << "state before linearization is: " << f << std::endl;

    const TireForces F_front = getForceFront(x);
    const TireForces F_rear  = getForceRear(x);
    const Torques T = getTorques(x); 
    const double lr_t = getLengthRear(x);
    const double lf_t = getLengthFront(x);
    //const Acceleration a = getAcceleration(x);

    const TireForcesDerivatives dF_front = getForceFrontDerivatives(x);
    const TireForcesDerivatives dF_rear  = getForceRearDerivatives(x);
    const TorquesDerivatives dT = getTorquesDerivatives(x);

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

    // f4 = vx
    const double df4_dvx = (1/param_.m)*(dF_rear.dF_x_vx-dF_front.dF_y_vx*std::sin(delta) + dF_front.dF_x_vx*std::cos(delta)); 
    const double df4_dvy = (1/param_.m)*(dF_rear.dF_x_vy-dF_front.dF_y_vy*std::sin(delta) + dF_front.dF_x_vy*std::cos(delta)) + r; 
    const double df4_dr = (1/param_.m)*(dF_rear.dF_x_r-dF_front.dF_y_r*std::sin(delta) + dF_front.dF_x_r*std::cos(delta)) + vy;
    const double df4_drwf = (1/param_.m)*(dF_rear.dF_x_rwf-dF_front.dF_y_rwf*std::sin(delta) + dF_front.dF_x_rwf*std::cos(delta));
    const double df4_drwr = (1/param_.m)*(dF_rear.dF_x_rwr-dF_front.dF_y_rwr*std::sin(delta) + dF_front.dF_x_rwr*std::cos(delta));
    const double df4_ddelta = (1/param_.m)*(dF_rear.dF_x_delta-dF_front.dF_y_delta*std::sin(delta) - F_front.F_y*std::cos(delta) + dF_front.dF_x_delta*std::cos(delta) - F_front.F_x*std::sin(delta));

    // f5 = vy
    const double df5_dvx = (1/param_.m)*(dF_rear.dF_y_vx+dF_front.dF_y_vx*std::cos(delta) + dF_front.dF_x_vx*std::sin(delta)) - r; 
    const double df5_dvy = (1/param_.m)*(dF_rear.dF_y_vy+dF_front.dF_y_vy*std::cos(delta) + dF_front.dF_x_vy*std::sin(delta)); 
    const double df5_dr = (1/param_.m)*(dF_rear.dF_y_r+dF_front.dF_y_r*std::cos(delta) + dF_front.dF_x_r*std::sin(delta)) - vx;
    const double df5_drwf = (1/param_.m)*(dF_rear.dF_y_rwf+dF_front.dF_y_rwf*std::cos(delta) + dF_front.dF_x_rwf*std::sin(delta));
    const double df5_drwr = (1/param_.m)*(dF_rear.dF_y_rwr+dF_front.dF_y_rwr*std::cos(delta) + dF_front.dF_x_rwr*std::sin(delta));
    const double df5_ddelta = (1/param_.m)*(dF_rear.dF_y_delta+dF_front.dF_y_delta*std::cos(delta) - F_front.F_y*std::sin(delta) + dF_front.dF_x_delta*std::sin(delta) + F_front.F_x*std::cos(delta));

    // f6 = rdot;
    const double df6_dvx     = (1.0/param_.Iz)*(dF_front.dF_y_vx*lf_t*std::cos(delta)    - dF_rear.dF_y_vx*lr_t+ dF_front.dF_x_vx*lf_t*std::sin(delta));
    const double df6_dvy     = (1.0/param_.Iz)*(dF_front.dF_y_vy*lf_t*std::cos(delta)    - dF_rear.dF_y_vy*lr_t+ dF_front.dF_x_vy*lf_t*std::sin(delta));
    const double df6_dr      = (1.0/param_.Iz)*(dF_front.dF_y_r*lf_t*std::cos(delta)    - dF_rear.dF_y_r*lr_t+ dF_front.dF_x_r*lf_t*std::sin(delta));
    const double df6_drwf     = (1.0/param_.Iz)*(dF_front.dF_y_rwf*lf_t*std::cos(delta)    - dF_rear.dF_y_rwf*lr_t+ dF_front.dF_x_rwf*lf_t*std::sin(delta));
    const double df6_drwr     = (1.0/param_.Iz)*(dF_front.dF_y_rwr*lf_t*std::cos(delta)    - dF_rear.dF_y_rwr*lr_t+ dF_front.dF_x_rwr*lf_t*std::sin(delta));
    const double df6_ddelta  = (1.0/param_.Iz)*(dF_front.dF_y_delta*lf_t*std::cos(delta) - F_front.F_y*lf_t*std::sin(delta) - dF_rear.dF_y_delta*lr_t + dF_front.dF_x_delta*lf_t*std::sin(delta)+F_front.F_x*lf_t*std::cos(delta));

    //f7 = rwr_dot
    const double df7_dvx = (1.0/param_.Iw)*(-param_.Rf*dF_front.dF_x_vx);
    const double df7_dvy =(1.0/param_.Iw)*(-param_.Rf*dF_front.dF_x_vy);
    const double df7_dr =(1.0/param_.Iw)*(-param_.Rf*dF_front.dF_x_r);
    const double df7_drwf =(1.0/param_.Iw)*(-param_.Rf*dF_front.dF_x_rwf);
    const double df7_ddelta = (1.0/param_.Iw)*(-param_.Rf*dF_front.dF_x_delta);
    const double df7_dD  =  (1.0/param_.Iw)*(-dT.dTBf_D);

    //f8 = rwf_dot
    const double df8_dvx = (1.0/param_.Iw)*(-param_.Rr*dF_rear.dF_x_vx);
    const double df8_drwr = (1.0/param_.Iw)*(-param_.Rr*dF_rear.dF_x_rwr);
    const double df8_dD  = (1.0/param_.Iw)*(dT.dTG_D - dT.dTBr_D);

    // Jacobians
    // lines -> equation
    // column -> state parameter 
    // Matrix A
    // Column 1 -> X
    // all zero
    // Column 2 -> Y
    // all zero
    // Column 3 -> phi
    A_c(0,2) = df1_dphi;
    A_c(1,2) = df2_dphi;
    // Column 4 -> vx
    A_c(0,3) = df1_dvx;
    A_c(1,3) = df2_dvx;
    A_c(3,3) = df4_dvx;
    A_c(4,3) = df5_dvx;
    A_c(5,3) = df6_dvx;
    A_c(6,3) = df7_dvx;
    A_c(7,3) = df8_dvx;
    // Column 5 -> vy
    A_c(0,4) = df1_dvy;
    A_c(1,4) = df2_dvy;
    A_c(3,4) = df4_dvy;
    A_c(4,4) = df5_dvy;
    A_c(5,4) = df6_dvy;
    A_c(6,4) = df7_dvy;
    // Column 6 -> r
    A_c(2,5) = df3_dr;
    A_c(3,5) = df4_dr;
    A_c(4,5) = df5_dr;
    A_c(5,5) = df6_dr;
    A_c(6,5) = df7_dr;
    // Column 7 -> rwf
    A_c(3,6) = df4_drwf;
    A_c(4,6) = df5_drwf;
    A_c(5,6) = df6_drwf;
    A_c(6,6) = df7_drwf;
    // Column 8 -> rwr
    A_c(3,7) = df4_drwr;
    A_c(4,7) = df5_drwr;
    A_c(5,7) = df6_drwr;
    A_c(7,7) = df8_drwr;
    // Column 9 -> vs 
    A_c(8,8) = 1.0;
    // all zero
    // Column 10 -> D
    A_c(6,9) = df7_dD;
    A_c(7,9) = df8_dD;
    //column 11 -> delta 
    A_c(3,10)= df4_ddelta;
    A_c(4,10)= df5_ddelta;
    A_c(5,10)= df6_ddelta;
    A_c(6,10)= df7_ddelta;
    //column 12 -> dVs
    A_c(8,11)=1.0;

    // Matrix B
    // Column 1
    B_c(9,0) = 1.0;
    // Column 2
    B_c(10,1) = 1.0;
    // Column 3
    B_c(11,2) = 1.0;

    //zero order term
    //std::cout << A_c << std::endl;

    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);
    
    //std::cout << "A matrix Jacobian is: " << A_c << std::endl;
    // std::cout << "B matrix Jacobian is: " << B_c << std::endl;
    // std::cout << "g matrix Jacobian is: "<< g_c << std::endl;

    //std::cout << "A matrix Jacobian is of shape: " << A_c.size() << std::endl;
    // std::cout << "B matrix Jacobian is of shape: " << B_c.size() << std::endl;
    // std::cout << "g matrix Jacobian is of shape: "<< g_c.size() << std::endl;
    
    return {A_c,B_c,g_c};
}

// Mixed RK4 - EXPM method
LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c, const State &x, const Input &u,const State &x_next) const
{
    // discretize the continuous time linear model \dot x = A x + B u + g using ZHO
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
    
    //std::cout << "A matrix discretized is: " << A_d << std::endl;
    //std::cout << "B matrix discretized is: " << B_d << std::endl;
    //std::cout << "g matrix discretized is: "<< g_d << std::endl;

    //std::cout << "A matrix discretized is of shape: " << A_d.size() << std::endl;
    //std::cout << "B matrix discretized is of shape: " << B_d.size() << std::endl;
    //std::cout << "g matrix discretized is of shape: "<< g_d.size() << std::endl;
    
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