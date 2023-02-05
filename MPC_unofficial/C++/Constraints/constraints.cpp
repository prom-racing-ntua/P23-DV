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

#include "constraints.h"
namespace mpcc{
Constraints::Constraints()
{   
    std::cout << "default constructor of constraints, not everything is initialized properly" << std::endl;
}

Constraints::Constraints(double Ts,const PathToJson &path) 
:model_(Ts,path),
param_(Param(path.param_path))
{
}

OneDConstraint Constraints::getTrackConstraints(const ArcLengthSpline &track,const State &x) const
{
    // given arc length s and the track -> compute linearized track constraints
    const double s = x.s;

    // X-Y point of the center line
    const Eigen::Vector2d pos_center = track.getPostion(s);
    const Eigen::Vector2d d_center   = track.getDerivative(s);
    // Tangent of center line at s
    const Eigen::Vector2d tan_center = {-d_center(1),d_center(0)};

    // inner and outer track boundary given left and right width of track
    // TODO make R_out and R_in dependent on s
    const Eigen::Vector2d pos_outer = pos_center + param_.r_out*tan_center;
    const Eigen::Vector2d pos_inner = pos_center - param_.r_in*tan_center;

    // Define track Jacobian as Perpendicular vector
    C_i_MPC C_track_constraint = C_i_MPC::Zero();
    C_track_constraint(0,0) = tan_center(0);
    C_track_constraint(0,1) = tan_center(1);
    // Compute bounds
    const double track_constraint_lower = tan_center(0)*pos_inner(0) + tan_center(1)*pos_inner(1);
    const double track_constraint_upper = tan_center(0)*pos_outer(0) + tan_center(1)*pos_outer(1);

    return {C_track_constraint,track_constraint_lower,track_constraint_upper};
}

// OneDConstraint Constraints::getTireConstraintRear(const State &x) const
// {
//     // compute tire friction elipse constraints
//     // (param_.E_long*Frx)^2 + Fry^2 <= (param_.E_eps*F_max)^2
//     const StateVector x_vec = stateToVector(x);
//     const TireForces f_rear = model_.getForceRear(x);
//     const NormalForces f_normal = model_.getForceNormal(x);

//     // compute tire friction constraint jacobean
//     const C_i_MPC C_tire_constraint = getTireConstraintRearJac(x);

//     // compute zero order term and max force
// //    const double tireCon0 = std::sqrt(std::pow(param_.e_long*f_rear.F_x,2) + std::pow(f_rear.F_y,2)); //zero order term
//     const double tireCon0 = std::pow(param_.e_long*f_rear.F_x/f_normal.F_N_rear,2) + std::pow(f_rear.F_y/f_normal.F_N_rear,2); //zero order term
//     const double maxForce = std::pow(param_.e_eps*param_.Dr/f_normal.F_N_rear,2);//param_.e_eps*param_.Dr;// //max allowed force

//     // set bounds given linearized constraint
//     // 0 <= 'Jac TC' (x - x0) + TC(x0) <= F_max
//     const double tire_constraint_lower = C_tire_constraint*x_vec-tireCon0;
//     const double tire_constraint_upper = maxForce+C_tire_constraint*x_vec-tireCon0;

//     return {C_tire_constraint,tire_constraint_lower,tire_constraint_upper};
// }

// C_i_MPC Constraints::getTireConstraintRearJac(const State &x) const
// {
//     // compute Jacobean of the tire constraints
//     const TireForces f_rear = model_.getForceRear(x);
//     const TireForcesDerivatives df_rear = model_.getForceRearDerivatives(x);
//     const NormalForces f_normal = model_.getForceNormal(x);

// //    const double TC = 2.0*std::sqrt(std::pow(param_.e_long*f_rear.F_x,2) + std::pow(f_rear.F_y,2));
// //
// //    // Tire constraint derivatives
// //    // TC = (param_.E_long*Frx)^2 + Fry^2
// //    const double dTC_dvx = (2.0*param_.e_long*f_rear.F_x*df_rear.dF_x_vx + 2.0*f_rear.F_y*df_rear.dF_y_vx)/TC;
// //    const double dTC_dvy = (2.0*f_rear.F_y*df_rear.dF_y_vy)/TC;
// //    const double dTC_dr  = (2.0*f_rear.F_y*df_rear.dF_y_r)/TC;
// //    const double dTC_dD  = (2.0*param_.e_long*f_rear.F_x*df_rear.dF_x_D)/TC;

//     const double TC = std::pow(param_.e_long*f_rear.F_x/f_normal.F_N_rear,2) + std::pow(f_rear.F_y/f_normal.F_N_rear,2);

//     // Tire constraint derivatives
//     // TC = (param_.e_long*Frx)^2 + Fry^2
//     const double dTC_dvx = (2.0*param_.e_long*f_rear.F_x/f_normal.F_N_rear*param_.e_long*df_rear.dF_x_vx/f_normal.F_N_rear +
//                             2.0*f_rear.F_y/f_normal.F_N_rear*df_rear.dF_y_vx/f_normal.F_N_rear);
//     const double dTC_dvy = (2.0*f_rear.F_y/f_normal.F_N_rear*df_rear.dF_y_vy/f_normal.F_N_rear);
//     const double dTC_dr  = (2.0*f_rear.F_y/f_normal.F_N_rear*df_rear.dF_y_r/f_normal.F_N_rear);
//     const double dTC_dD  = (2.0*param_.e_long*f_rear.F_x/f_normal.F_N_rear*param_.e_long*df_rear.dF_x_D/f_normal.F_N_rear);

//     // Copy partial derivatives in jacobean matrix
//     C_i_MPC Jac_tireCon = C_i_MPC::Zero();
//     Jac_tireCon(si_index.vx) = dTC_dvx;
//     Jac_tireCon(si_index.vy) = dTC_dvy;
//     Jac_tireCon(si_index.r)  = dTC_dr;
//     Jac_tireCon(si_index.D)  = dTC_dD;
//     //std::cout << "tyre constraint jacobian is:" <<  Jac_tireCon << std::endl;

//     return Jac_tireCon;
// }

MuConstraints Constraints::getEllipseParams(const double &Fz) const {
	//Using tire magic formula
	const double Fz0=1112.0554070627252;
	const double dfz=(Fz-Fz0)/Fz0;
	const double C_tire=0.66;
	const double mx_max=C_tire*(2.21891927-1.36151651e-07*dfz);
	const double my_max=C_tire*(2.46810824-0.21654031*dfz);
    // std::cout << "as and bs are: " << mx_max << " " << my_max << std::endl;
	return {mx_max,my_max};
}

OneDConstraint Constraints::getTireConstraintRear(const State &x) const
{
    // compute tire friction elipse constraints
    // (param_.E_long*Frx)^2 + Fry^2 <= (param_.E_eps*F_max)^2
    const StateVector x_vec = stateToVector(x);
    const TireForces f_rear = model_.getForceRear(x);
    const NormalForces f_normal = model_.getForceNormal(x);

    // compute tire friction constraint jacobean
    const C_i_MPC C_tire_constraint = getTireConstraintRearJac(x);
    const MuConstraints mu = getEllipseParams(f_rear.F_z);
    const MuConstraints mu0 = getEllipseParams(0);

    const double a = mu.mx_max;
    const double b = mu.my_max;
    const double a0 = mu0.mx_max;
    const double b0 = mu0.my_max;

    // compute zero order term and max force
    //const double tireCon0 = std::sqrt(std::pow(param_.e_long*f_rear.F_x,2) + std::pow(f_rear.F_y,2)); //zero order term
    const double tireCon0 = std::pow(f_rear.F_x/(a*f_rear.F_z),2) + std::pow(f_rear.F_y/(b*f_rear.F_z),2); //zero order term
    const double maxForce = std::pow(1,2);//param_.e_eps*param_.Dr;// //max allowed force
    // std::cout << "constraints of forces rear are: " << f_rear.F_z << " " << a << " " << b << " " << a0 << " " << b0  << std::endl;

    // set bounds given linearized constraint
    // 0 <= 'Jac TC' (x - x0) + TC(x0) <= F_max
    const double tire_constraint_lower = C_tire_constraint*x_vec-tireCon0;
    const double tire_constraint_upper = maxForce+C_tire_constraint*x_vec-tireCon0;

    return {C_tire_constraint,tire_constraint_lower,tire_constraint_upper};
}

C_i_MPC Constraints::getTireConstraintRearJac(const State &x) const
{
    /// compute tire friction elipse constraints
    // (param_.E_long*Frx)^2 + Fry^2 <= (param_.E_eps*F_max)^2
    const StateVector x_vec = stateToVector(x);
    const TireForces f_rear = model_.getForceRear(x);
    const NormalForces f_normal = model_.getForceNormal(x);
    const TireForcesDerivatives df_rear=model_.getForceRearDerivatives(x);

    // compute tire friction constraint jacobean
    const MuConstraints mu = getEllipseParams(f_rear.F_z);

    const double a = mu.mx_max;
    const double b = mu.my_max;

    const double TC = std::pow(f_rear.F_x/a,2) + std::pow(f_rear.F_y/b,2);

    // Tire constraint derivatives
    // TC = (param_.e_long*Frx)^2 + Fry^2   
    const double dTC_dvx = (2.0*f_rear.F_x/(a*f_rear.F_z))*(df_rear.dF_x_vx) + (2.0*f_rear.F_y/(b*f_rear.F_z))*(df_rear.dF_y_vx);
    // const double dTC_dvx = ((2.0*f_rear.F_x)/(a*(std::pow(f_rear.F_z,3))))*(df_rear.dF_x_vx*f_rear.F_z - f_rear.F_x*df_rear.dF_z_vx) + ((2.0*f_rear.F_y)/(b*(std::pow(f_rear.F_z,3))))*(df_rear.dF_y_vx*f_rear.F_z - f_rear.F_y*df_rear.dF_z_vx); //TODO: include derivatives of ellipse function
    const double dTC_dvy = (2.0*f_rear.F_x/(a*f_rear.F_z))*(df_rear.dF_x_vy) + (2.0*f_rear.F_y/(b*f_rear.F_z))*(df_rear.dF_y_vy);
    const double dTC_dr  = (2.0*f_rear.F_x/(a*f_rear.F_z))*(df_rear.dF_x_r) + (2.0*f_rear.F_y/(b*f_rear.F_z))*(df_rear.dF_y_r);
    const double dTC_drwr = (2.0*f_rear.F_x/(a*f_rear.F_z))*(df_rear.dF_x_rwr) + (2.0*f_rear.F_y/(b*f_rear.F_z))*(df_rear.dF_y_rwr);

    // Copy partial derivatives in jacobean matrix
    C_i_MPC Jac_tireCon = C_i_MPC::Zero();
    Jac_tireCon(si_index.vx) = dTC_dvx;
    Jac_tireCon(si_index.vy) = dTC_dvy;
    Jac_tireCon(si_index.r)  = dTC_dr;
    Jac_tireCon(si_index.rwf)  = dTC_drwr;

    return Jac_tireCon;
}



OneDConstraint Constraints::getAlphaConstraintFront(const State &x) const
{
    // compute linearized slip angle constraints
    // -alpha_max <= alpha_f <= alpha_max
    const StateVector x_vec = stateToVector(x);
    SlipAngles sa = model_.getSlipAngles(x);
    // compute the jacobean of alpha_f
    const C_i_MPC C_alpha_constraint = getAlphaConstraintFrontJac(x);
    // compute the bounds given the Tylor series expansion
    const double alpha_constraint_lower = -param_.max_alpha; //-sa.sa_f+C_alpha_constraint*x_vec;
    const double alpha_constraint_upper =  param_.max_alpha;  //-sa.sa_f+C_alpha_constraint*x_vec;
    // std::cout << "slip angle constraint is: " << alpha_constraint_lower << " " << sa.sa_f << " " << alpha_constraint_upper  << std::endl;

    return {C_alpha_constraint,alpha_constraint_lower,alpha_constraint_upper};
}

C_i_MPC Constraints::getAlphaConstraintFrontJac(const State &x) const
{
    // compute the alpha_f jacobian
    const double vx     = x.vx;
    const double vy     = x.vy;
    const double r      = x.r;
    const double delta  = x.delta;

    C_i_MPC Jac_alphaCon;
    const double lf = model_.getLengthFront(x);
    const double lr = model_.getLengthRear(x);
    SlipAngleDerivatives dsa = model_.getSlipAngleDerivatives(x);
    // alpha_f = -atan(vy+r*param_.lf/vx) + delta;
    // compute partial derivatives
    const double dalpha_f_dvx    = dsa.dsaf_vx;
    const double dalpha_f_dvy    = dsa.dsaf_vy;
    const double dalpha_f_dr     = dsa.dsaf_r;
    const double dalpha_f_ddelta = dsa.dsaf_delta;

    // Copy partial derivatives in jacobean matrix
    Jac_alphaCon.setZero();
    Jac_alphaCon(si_index.vx)   = dalpha_f_dvx;
    Jac_alphaCon(si_index.vy)   = dalpha_f_dvy;
    Jac_alphaCon(si_index.r)    = dalpha_f_dr;
    Jac_alphaCon(si_index.delta)= dalpha_f_ddelta;

    return Jac_alphaCon;

}

ConstrainsMatrix Constraints::getConstraints(const ArcLengthSpline &track,const State &x,const Input &u) const
{
    // compute all the polytopic state constraints
    // compute the three constraints

    ConstrainsMatrix constrains_matrix;
    const OneDConstraint track_constraints = getTrackConstraints(track,x);
    //std::cout << "track constraints done" << std::endl;
    const OneDConstraint tire_constraints_rear = getTireConstraintRear(x);
    //std::cout << "tire constraints done" << std::endl;
    const OneDConstraint alpha_constraints_front = getAlphaConstraintFront(x);
    //std::cout << "alpha constraints done" << std::endl;

    C_MPC C_constrains_matrix;
    d_MPC dl_constrains_matrix;
    d_MPC du_constrains_matrix;

    C_constrains_matrix.row(si_index.con_track) = track_constraints.C_i;
    dl_constrains_matrix(si_index.con_track) = track_constraints.dl_i;
    du_constrains_matrix(si_index.con_track) = track_constraints.du_i;

    C_constrains_matrix.row(si_index.con_tire) = tire_constraints_rear.C_i;
    dl_constrains_matrix(si_index.con_tire) = tire_constraints_rear.dl_i;
    du_constrains_matrix(si_index.con_tire) = tire_constraints_rear.du_i;

    C_constrains_matrix.row(si_index.con_alpha) = alpha_constraints_front.C_i;
    dl_constrains_matrix(si_index.con_alpha) = alpha_constraints_front.dl_i;
    du_constrains_matrix(si_index.con_alpha) = alpha_constraints_front.du_i;

    // TODO consider the zero order term directly in the functions construdcing the constraints
    dl_constrains_matrix = dl_constrains_matrix -  C_constrains_matrix*stateToVector(x);   
    du_constrains_matrix = du_constrains_matrix -  C_constrains_matrix*stateToVector(x);  

    return {C_constrains_matrix,D_MPC::Zero(),dl_constrains_matrix,du_constrains_matrix};
}
}
