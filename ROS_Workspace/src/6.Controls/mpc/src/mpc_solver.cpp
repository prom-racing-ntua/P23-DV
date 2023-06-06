#include "mpc_solver.h"
using namespace path_planning;
using namespace mpc;
namespace mpc {
    MpcSolver::MpcSolver() {
        std::cout <<"mpc solver constructed" << std::endl;
    }

    MpcSolver::~MpcSolver(){
        std::cout << "mpc solver destroyed" << std::endl;
    }

double MpcSolver::getFy(double Fz,double sa) {
    const double C_tire=0.66;
    const double P [] = {1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04};
    const double ex=0.00001;

    const double C=P[0];
    const double D=P[1]*std::pow(Fz,2)+P[2]*Fz;
    const double BCD = P[3]*std::sin(P[4]*std::atan(P[5]*Fz));
    const double B = (BCD)/(C*D +ex);
    const double Fy_out = D*C_tire * std::sin(C * std::atan(B *sa));
    return Fy_out;
}

NormalForces MpcSolver::getFz(const double X[X_SIZE]) {
    const double Ffz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2));
    const double Frz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2));
    return {Ffz,Frz};
}

SlipAngles MpcSolver::getSlipAngles(const double X[X_SIZE]){
    double saf=std::atan((X[4]+l_f*X[5])/(X[3]+1e-3)) - X[7];
    double sar=std::atan((X[4]-l_r*X[5])/(X[3]+1e-3));
    return {saf,sar};
}

MuConstraints MpcSolver::getEllipseParams(const double &Fz) {
	//Using tire magic formula
	const double Fz0=1112.0554070627252;
	const double dfz=(Fz-Fz0)/Fz0;
	const double C_tire=0.66;
	const double mx_max=C_tire*(2.21891927-1.36151651e-07*dfz);
	const double my_max=C_tire*(2.46810824-0.21654031*dfz);
	return {mx_max,my_max};
}

    void MpcSolver::getF(double X[X_SIZE], double U[U_SIZE], double (&kappa)[X_SIZE]) {
        const double temp=(X[3]-umin)/(umax-umin);
        double double_1 = custom_max(temp,0);
        double l_a = custom_min(double_1,1);
        SlipAngles sa = getSlipAngles(X);
        NormalForces Fz = getFz(X);
        double Fry = getFy(Fz.Ffz,sa.sar);
        double Ffy = getFy(Fz.Ffz,sa.saf);
        double Fdrag = 0.5*CdA*p_air*std::pow(X[3],2) + 0.03*(Fz.Frz+Fz.Ffz);
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

    double MpcSolver::custom_max(double aa, double bb){
    if(aa<bb){
        return bb;
    }
    else {
        return aa;
    }
    }

    double MpcSolver::custom_min(double aa, double bb){
        if(aa<bb){
            return aa;
        }
        else {
            return bb;
        }
    }

    void MpcSolver::Integrator()
    {
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
                X[i] += step_all;
            }
    }


void MpcSolver::Initialize_all_local() {
    for (int i = 0; i < Z_SIZE*LOOKAHEAD; i++) {
        params.x0[i] = 0.0;
        // std::cout << "i is: " << i << std::endl;
    }
    //X_array is: X  Y  phi vx vy  r F delta index
    // const double xinit_temp[9];
    if(known_track_) {
        const double xinit_temp_known[9] = {whole_track(0,0),whole_track(0,1),whole_track(0,2),0.0,0.0,0.0,F_init,0.0,0.0};
        for (int i = 0; i < X_SIZE; ++i) {
            X[i] = xinit_temp_known[i];
        }
    }
    else {
        std::cout << "initialized unknown" << std::endl;
        const double xinit_temp_unknown[9] = {params_array(0,0),params_array(0,1),params_array(0,2),0.0,0.0,0.0,F_init,0.0,0.0};
        for (int i = 0; i < X_SIZE; ++i) {
            X[i] = xinit_temp_unknown[i];
        }
    }
    std::cout << "finished initialization of parameters " << std::endl;
    std::cout << "x, y and phi_init are: " << X[0] << " " << X[1] << " " << X[2] << std::endl;
}

void MpcSolver::UpdateFromLastIteration() {
    for (int j = 0; j < X_SIZE; ++j) params.xinit[j] = X[j];
    if(!simulation_) {
        X[0] = pose_struct.x;
        X[1] = pose_struct.y;
        X[2] = pose_struct.theta;
        X[3] = vel_struct.velocity_x;
        X[4] = vel_struct.velocity_y;
        X[5] = vel_struct.yaw_rate;
    }
    std::cout << "pose at start is: " << X[0] << " " << X[1] << " " << X[2] << std::endl;
    std::cout << "velocity at start is: " << X[3] << " " << X[4] << " " << X[5] << std::endl; 
}

void MpcSolver::generateFirstPoint() {
    error_eucl.clear();
    error_ver.clear();
    for(int i = 0; i<whole_track.rows(); ++i) { //second to second to last for ds comparison
        error_eucl.push_back(std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0)));
    }
    closest_index_eucl = std::distance(error_eucl.begin(), std::min_element(error_eucl.begin(), error_eucl.end()));
    dist_eucl = error_eucl[closest_index_eucl];
    for(int j = 0 ; j< whole_track.rows(); ++j) {
        error_ver.push_back(std::abs(std::sin(whole_track(j,2))*(X[0]-whole_track(j,0)) - std::cos(whole_track(j,2))*(X[1]-whole_track(j,1)))); 
    }
    closest_index_ver = std::distance(error_ver.begin(), std::min_element(error_ver.begin(), error_ver.end()));
    dist_ver = error_ver[closest_index_ver];
    std::cout << "eucleidian and vertical distances are: " << dist_eucl << " " << dist_ver << std::endl;
    std::cout << "eucleidian and vertical indices are: " << closest_index_eucl << " " <<  closest_index_ver << std::endl;
    std::vector<double> target_lengths_temp;
    target_lengths_temp.push_back(0.0);
    for (int i{ 1 }; i <= closest_index_eucl; i++) {
        target_lengths_temp.push_back(target_lengths_temp[i - 1] + \
            std::sqrt(std::pow(whole_track(i, 0) - whole_track(i - 1, 0), 2) + std::pow(whole_track(i, 1) - whole_track(i - 1, 1), 2)));
    }
    double s_init = target_lengths_temp[closest_index_eucl];
    if(dist_eucl > distance_safe_) {
        emergency = 1;
        std::cout << "mpika emergency!!!" << std::endl;
        s_init += emergency_forward_;
    }
    if(s_init > sol) s_init = 0.0;
    s_array_final[0] = s_init;
    std::cout << "sinit is:" << s_init << std::endl;
    std::cout << "generated first point " << spline_final->getPoint(s_init/sol) << std::endl;
    }

void MpcSolver::generateFirstPointUnknown() {
    error_eucl.clear();
    error_ver.clear();
    for(int i = 0; i<params_array.rows(); ++i) { //second to second to last for ds comparison
        error_eucl.push_back(std::sqrt(std::pow((X[0]-params_array(i,0)),2.0) + std::pow((X[1] - params_array(i,1)),2.0)));
    }
    closest_index_eucl = std::distance(error_eucl.begin(), std::min_element(error_eucl.begin(), error_eucl.end()));
    dist_eucl = error_eucl[closest_index_eucl];
    for(int j = 0 ; j< params_array.rows(); ++j) {
        error_ver.push_back(std::abs(std::sin(params_array(j,2))*(X[0]-params_array(j,0)) - std::cos(params_array(j,2))*(X[1]-params_array(j,1)))); 
    }
    closest_index_ver = std::distance(error_ver.begin(), std::min_element(error_ver.begin(), error_ver.end()));
    dist_ver = error_ver[closest_index_ver];
    std::cout << "eucleidian and vertical distances are: " << dist_eucl << " " << dist_ver << std::endl;
    std::cout << "eucleidian and vertical indices are: " << closest_index_eucl << " " <<  closest_index_ver << std::endl;
    std::vector<double> target_lengths_temp;
    target_lengths_temp.push_back(0.0);
    double s_init = 0.0;
    if(closest_index_eucl>0) {
        for (int i{ 1 }; i <= closest_index_eucl; i++) {
            target_lengths_temp.push_back(target_lengths_temp[i - 1] + \
                std::sqrt(std::pow(params_array(i, 0) - params_array(i - 1, 0), 2) + std::pow(params_array(i, 1) - params_array(i - 1, 1), 2)));
        }
    s_init = target_lengths_temp[closest_index_eucl];
    }
    else s_init = 0.0;
    std::cout << "found s_init" << std::endl;
    if(dist_eucl > distance_safe_) {
        emergency = 1;
        std::cout << "mpika emergency!!!" << std::endl;
        s_init += emergency_forward_;
    }
    if(s_init > sol) s_init = sol;
    s_array_final[0] = s_init;
    int ind_of_closest = (s_init/sol)*40;
    std::cout << "generated first point " << s_init << " " << params_array(ind_of_closest,0) << " " << params_array(ind_of_closest,1) << std::endl;
    }

    PointsData MpcSolver::getSplineDataLocal(double parameters[LOOKAHEAD]) {
        // Iterate through all points and data of interest (X,Y,tang,curv)
        // int rows_ = (int)sizeof(parameters);
        std::cout << "started spline data local" << std::endl;
        PointsData spline_data{LOOKAHEAD,4};
        NormalForces Fz = getFz(X);
        MuConstraints mu = getEllipseParams(Fz.Frz);
        std::cout << "got Fz,mu " << Fz.Frz << " " << Fz.Ffz << " " <<  mu.mx_max << " "<<mu.my_max  << std::endl;
        for (long int i{ 0 }; i < LOOKAHEAD; i++) {
            double param = parameters[i];
            Point temp1 = spline_final->getPoint(param);
            double temp2 = spline_final->getTangent(param);
            double temp3 = spline_final->getVelocity(param,mu.my_max,v_limit_);
            if(i==0) std::cout << "closest spline point is: " << temp1(0) << " " << temp1(1) << std::endl; 
            spline_data(i, 0) = temp1(0);
            spline_data(i, 1) = temp1(1);
            spline_data(i, 2) = temp2;
            spline_data(i, 3) = temp3;
        }
        return spline_data;
        std::cout << "got spline data local" << std::endl;
    }

    void MpcSolver::writeParamsKnown(int global_int) {
        std::cout << "started writing of known_params" << std::endl;
        for (int i = 1; i < LOOKAHEAD; ++i){
            if(emergency || global_int==-1 ) s_array_final[i] = s_array_final[i-1] + s_interval_;
            else s_array_final[i] = s_array_final[i-1] + (ds_vector[i-1]);
            if(s_array_final[i]>sol) {
                std::cout << "finished lap" << std::endl;
                s_array_final[i]=sol;
            }
        }
        for(int i=0; i<LOOKAHEAD; ++i) s_array_final[i]=s_array_final[i]/sol;
        std::cout << "finished generation of s_profile" << std::endl;
        params_array = getSplineDataLocal(s_array_final);
        copyToParameters();
    }

    void MpcSolver::writeParamsUnknown() {
        std::cout << "started writing of unknown_params" << std::endl;
        s_array_final[0] = (float)s_array_final[0];
        for (int i = 1; i < LOOKAHEAD; ++i){
            s_array_final[i] = s_array_final[i-1] + s_interval_;
            if(s_array_final[i]>sol) {
                std::cout << "finished spline" << std::endl;
                s_array_final[i]=sol;
            }
        }
        for(int i=0; i<LOOKAHEAD; ++i) s_array_final[i]=s_array_final[i]/sol;
        std::cout << "finished generation of s_profile" << std::endl;
        std::cout << "three s_final params are: " << s_array_final[0] << " " << s_array_final[1] << " " << s_array_final[5];
        params_array = getSplineDataLocal(s_array_final);
        copyToParameters();
    }

    void MpcSolver::copyToParameters() {
        for(int k = 0; k < 4*LOOKAHEAD; ++k) { 
            int mod_ = k%4;
            if (mod_ == 0) {
                params.all_parameters[k] = params_array(int(k/4),0); 
                // std::cout << "added X " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if(mod_ == 1) {
                params.all_parameters[k] = params_array(int((k-1)/4),1);
                // std::cout << "added Y " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if(mod_ == 2) {
                params.all_parameters[k] = params_array(int((k-2)/4),2);
                // std::cout << "added phi " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if (mod_ == 3) {
                params.all_parameters[k] = params_array(int((k-3)/4),3);
                // std::cout << "added curv " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
        }
        std::cout << "finished writing of all unknown_params" << std::endl;
    }

    void MpcSolver::writeLookaheadArray1() {
        s_vector.clear();
        s_vector.push_back(output.x01[2]);
        s_vector.push_back(output.x02[2]);
        s_vector.push_back(output.x03[2]);
        s_vector.push_back(output.x04[2]);
        s_vector.push_back(output.x05[2]);
        s_vector.push_back(output.x06[2]);
        s_vector.push_back(output.x07[2]);
        s_vector.push_back(output.x08[2]);
        s_vector.push_back(output.x09[2]);
        s_vector.push_back(output.x10[2]);
        s_vector.push_back(output.x11[2]);
        s_vector.push_back(output.x12[2]);
        s_vector.push_back(output.x13[2]);
        s_vector.push_back(output.x14[2]);
        s_vector.push_back(output.x15[2]);
        s_vector.push_back(output.x16[2]);
        s_vector.push_back(output.x17[2]);
        s_vector.push_back(output.x18[2]);
        s_vector.push_back(output.x19[2]);
        s_vector.push_back(output.x20[2]);
        s_vector.push_back(output.x21[2]);
        s_vector.push_back(output.x22[2]);
        s_vector.push_back(output.x23[2]);
        s_vector.push_back(output.x24[2]);
        s_vector.push_back(output.x25[2]);
        s_vector.push_back(output.x26[2]);
        s_vector.push_back(output.x27[2]);
        s_vector.push_back(output.x28[2]);
        s_vector.push_back(output.x29[2]);
        s_vector.push_back(output.x30[2]);
        s_vector.push_back(output.x31[2]);
        s_vector.push_back(output.x32[2]);
        s_vector.push_back(output.x33[2]);
        s_vector.push_back(output.x34[2]);
        s_vector.push_back(output.x35[2]);
        s_vector.push_back(output.x36[2]);
        s_vector.push_back(output.x37[2]);
        s_vector.push_back(output.x38[2]);
        s_vector.push_back(output.x39[2]);
        s_vector.push_back(output.x40[2]);
    }

    void MpcSolver::writeLookaheadArray2() {
        ds_vector.clear();
        ds_vector.push_back(output.x01[2]);
        ds_vector.push_back(output.x02[2]);
        ds_vector.push_back(output.x03[2]);
        ds_vector.push_back(output.x04[2]);
        ds_vector.push_back(output.x05[2]);
        ds_vector.push_back(output.x06[2]);
        ds_vector.push_back(output.x07[2]);
        ds_vector.push_back(output.x08[2]);
        ds_vector.push_back(output.x09[2]);
        ds_vector.push_back(output.x10[2]);
        ds_vector.push_back(output.x11[2]);
        ds_vector.push_back(output.x12[2]);
        ds_vector.push_back(output.x13[2]);
        ds_vector.push_back(output.x14[2]);
        ds_vector.push_back(output.x15[2]);
        ds_vector.push_back(output.x16[2]);
        ds_vector.push_back(output.x17[2]);
        ds_vector.push_back(output.x18[2]);
        ds_vector.push_back(output.x19[2]);
        ds_vector.push_back(output.x20[2]);
        ds_vector.push_back(output.x21[2]);
        ds_vector.push_back(output.x22[2]);
        ds_vector.push_back(output.x23[2]);
        ds_vector.push_back(output.x24[2]);
        ds_vector.push_back(output.x25[2]);
        ds_vector.push_back(output.x26[2]);
        ds_vector.push_back(output.x27[2]);
        ds_vector.push_back(output.x28[2]);
        ds_vector.push_back(output.x29[2]);
        ds_vector.push_back(output.x30[2]);
        ds_vector.push_back(output.x31[2]);
        ds_vector.push_back(output.x32[2]);
        ds_vector.push_back(output.x33[2]);
        ds_vector.push_back(output.x34[2]);
        ds_vector.push_back(output.x35[2]);
        ds_vector.push_back(output.x36[2]);
        ds_vector.push_back(output.x37[2]);
        ds_vector.push_back(output.x38[2]);
        ds_vector.push_back(output.x39[2]);
        ds_vector.push_back(output.x40[2]);
        for (int i = 0; i < LOOKAHEAD; ++i){
            if(ds_vector[i]>s_space_max) ds_vector[i]=s_space_max;
            if(ds_vector[i]<s_space_min) ds_vector[i]=s_space_min;
        }
    }

    void MpcSolver::generateOutput() { 
          //dF ddelta dindex
          for(int k = 0; k<3; k++){
              U[k]=output.x02[k];
          }
          writeLookaheadArray1();
          writeLookaheadArray2();
          std::cout << "U final array is: " << U[0] << " " << U[1] << " " << U[2] << std::endl;
          std::cout << "X,Y,phi is: " << X[0] << " " << X[1] << " " << X[2] << std::endl;  
          std::cout << "vy and r is: " << X[3] << " " << X[4] << " " << X[5] << std::endl;        
          //define message to ROS2
          output_struct.speed_target = (int)(5.0);
          output_struct.speed_actual = (int)(vel_struct.velocity_x);
          output_struct.motor_torque_target = (float)(X[6]*Rw/(gr*eff));
          output_struct.steering_angle_target = (float)(X[7]);
          output_struct.brake_pressure_target = (bool)(0);
          Integrator();
        }

    int MpcSolver::callSolver(int global_int) {
        exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, NULL, extfunc_eval);
        if (exitflag == 1) {
            printf("\n\nFORCESNLPsolver returned optimal solution at step %d. Exiting.\n", global_int);
        }
        else {
            std::cout << "something went wrong. Needs to be handled" << std::endl;
        }
        return exitflag;
    }
} //namespace mpc