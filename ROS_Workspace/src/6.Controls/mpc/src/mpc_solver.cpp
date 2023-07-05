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
    double a_temp = (X[6]-0.5*CdA*p_air*std::pow(X[3],2))/m;
    // std::cout << "acceleration temp is: " << a_temp << std::endl;
    double dw = (h_cog/(l_r+l_f))*(a_temp/g)*(m*g);
    // dw = 0.0;
    // std::cout << "weight transfer variable is: " << dw << std::endl;
    const double Ffz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2)) - dw;
    const double Frz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2)) + dw;
    return {Ffz,Frz};
}

SlipAngles MpcSolver::getSlipAngles(const double X[X_SIZE]) {
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
    l_a = 1; //ignoring model

    kappa[0] = (1)*(X[3]*std::cos(X[2]) - X[4]*std::sin(X[2])) + (0)*(X[3]*std::cos(X[2] + beta));  
    kappa[1] = (1)*(X[3]*std::sin(X[2]) + X[4]*std::cos(X[2])) + (0)*(X[3]*std::sin(X[2] + beta));
    kappa[2] = X[5];
    kappa[3] = (1-l_a)*((X[6] - Fdrag)/ m) + (l_a)*((X[6] - Fdrag + Ffy*std::sin(X[7]) + m*X[4]*X[5])/ m); //vxdot
    kappa[4] = (1-l_a)*((l_r/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*(((-X[3]*X[5]) + (Fry + Ffy*std::cos(X[7])))/(1.0*m));
    kappa[5] = (1-l_a)*((1/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*((Ffy*l_f*std::cos(X[7]) - Fry*l_r)/(1.0*Iz));
    kappa[6] = U[0];
    kappa[7] = U[1];
    kappa[8] = U[2]; 
}

    double MpcSolver::custom_max(double aa, double bb){
        if(aa<bb) return bb;
        else return aa;
    }

    double MpcSolver::custom_min(double aa, double bb){
        if(aa<bb) return aa;
        else return bb;
    }

    void MpcSolver::Integrator() {
        getF(X,U,k1);
        for (int i = 0; i<X_SIZE;++i) {
            X2[i] = X[i] +(dt/2)*k1[i];
        }
        getF(X2,U,k2);
        for (int i = 0; i<X_SIZE;++i) {
            X3[i] = X[i] +(dt/2)*k2[i];
        }
        getF(X3,U,k3);
        for (int i = 0; i<X_SIZE;++i) {
            X4[i] = X[i] + dt*k3[i];
        }
        getF(X4,U,k4);
        for (int i = 0; i<X_SIZE;++i) {
            double step_all = dt*((k1[i]/6) +(k2[i]/3) +(k3[i]/3) + (k4[i]/6));
            // std::cout << "step all is: " << step_all << std::endl;
            X[i] += step_all;
        }
    }


void MpcSolver::Initialize_all_local() {
    s_array_final.resize(lookahead_,0); //resize of array
    for (int i = 0; i < Z_SIZE*lookahead_; i++) params.x0[i] = 0.0;
    //X_array is: X  Y  phi vx vy  r F delta index
    // const double xinit_temp[9];
    if(known_track_) {
        const double xinit_temp_known[9] = {start_point.x,start_point.y,whole_track(0,2),0.1,0.0,0.0,F_init,0.0,0.0};
        for (int i = 0; i < X_SIZE; ++i) X[i] = xinit_temp_known[i];
    }
    else {
        std::cout << "initialized unknown" << std::endl;
        const double xinit_temp_unknown[9] = {start_point.x,start_point.y,whole_track(0,2),0.1,0.0,0.0,F_init,0.0,0.0};
        for (int i = 0; i < X_SIZE; ++i) X[i] = xinit_temp_unknown[i];
    }
    std::cout << "finished initialization of parameters " << std::endl;
    std::cout << "x, y and phi_init are: " << X[0] << " " << X[1] << " " << X[2] << std::endl;
}

void MpcSolver::UpdateFromLastIteration() {
    for (int j = 0; j < X_SIZE; ++j) params.xinit[j] = X[j];
    if(!simulation_) {
        std::cout << "I get feedback from car" << std::endl;
        X[0] = pose_struct.x;
        X[1] = pose_struct.y;
        X[2] = pose_struct.theta;
        X[3] = vel_struct.velocity_x;
        X[4] = vel_struct.velocity_y;
        X[5] = vel_struct.yaw_rate;
        if(mission_!="skidpad") lap_counter = lap_counter_official;
    }
    std::cout << "pose at start is: " << X[0] << " " << X[1] << " " << X[2] << std::endl;
    std::cout << "velocity at start is: " << X[3] << " " << X[4] << " " << X[5] << std::endl; 
}

void MpcSolver::generateFirstPoint() {
    error_eucl.clear();
    for(int i = 0; i<whole_track.rows(); ++i) { //second to second to last for ds comparison
        error_eucl.push_back(std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0)));
    }
    closest_index_eucl = std::distance(error_eucl.begin(), std::min_element(error_eucl.begin(), error_eucl.end())) + 1;
    dist_eucl = error_eucl[closest_index_eucl];
    // for(int j = 0 ; j< whole_track.rows(); ++j) {
    //     error_ver.push_back(std::abs(std::sin(whole_track(j,2))*(X[0]-whole_track(j,0)) - std::cos(whole_track(j,2))*(X[1]-whole_track(j,1)))); 
    // }
    // closest_index_ver = std::distance(error_ver.begin(), std::min_element(error_ver.begin(), error_ver.end()));
    // // dist_ver = error_ver[closest_index_ver];
    // std::cout << "eucleidian and vertical distances are: " << dist_eucl << " " << dist_ver << std::endl;
    // std::cout << "eucleidian and vertical indices are: " << closest_index_eucl << " " <<  closest_index_ver << std::endl;
    // std::vector<double> target_lengths_temp;
    // target_lengths_temp.push_back(0.0);
    // for (int i{ 1 }; i <= closest_index_eucl; i++) {
    //     target_lengths_temp.push_back(target_lengths_temp[i - 1] + 
    //         std::sqrt(std::pow(whole_track(i, 0) - whole_track(i - 1, 0), 2) + std::pow(whole_track(i, 1) - whole_track(i - 1, 1), 2)));
    // }
    double whole_length = (double) whole_track.rows();
    float percentage_ = closest_index_eucl/whole_length;
    double s_init = percentage_*sol;
    if(dist_eucl > distance_safe_) {
        emergency = 1;
        emergency_counter+=1;
        std::cout << "mpika emergency!!!" << std::endl;
        s_init += emergency_forward_;
    }
    else emergency = 0;
    if(s_init > sol) s_init = sol;
    s_array_final[0] = s_init;
    std::cout << "sinit and sol are: " << s_init << " " << sol << " (" << percentage_*100 << "%) " << std::endl;
    std::cout << "thetas from slam and splines are: " << pose_struct.theta*57.2958 << " " << (spline_final->getTangent(s_init/sol))*57.2958 << std::endl;
    std::cout << "generated first point " << spline_final->getPoint(s_init/sol) << " at distance " << dist_eucl << std::endl;
}

void MpcSolver::generateFirstPointUnknown() { //basically the same as known..
    error_eucl.clear();
    for(int i = 0; i<whole_track.rows(); ++i) { //second to second to last for ds comparison
        error_eucl.push_back(std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0)));
    }
    closest_index_eucl = std::distance(error_eucl.begin(), std::min_element(error_eucl.begin(), error_eucl.end()));
    std::cout << "closest_index_eucl is: " << closest_index_eucl << std::endl;
    dist_eucl = error_eucl[closest_index_eucl];
    double whole_length = (double) whole_track.rows();
    float percentage_ = closest_index_eucl/whole_length;
    double s_init = percentage_*sol;
    if(dist_eucl > distance_safe_) {
        emergency = 1;
        emergency_counter+=1;
        std::cout << "mpika emergency!!!" << std::endl;
        s_init += emergency_forward_;
    }
    else emergency = 0;
    if(s_init >= sol) s_init = sol-1e-3;
    s_array_final[0] = s_init;
    std::cout << "sinit and sol are: " << s_init << " " << sol << " (" << percentage_*100 << "%) " << std::endl;
    std::cout << "thetas from slam and splines are: " << pose_struct.theta*57.2958 << " " << (spline_final->getTangent(s_init/sol))*57.2958 << std::endl;
    std::cout << "generated first point " << spline_final->getPoint(s_init/sol) << " at distance " << dist_eucl << std::endl;
    }

    // PointsData MpcSolver::getSplineDataLocal(double parameters[lookahead_]) {
    PointsData MpcSolver::getSplineDataLocal(std::vector<double> parameters) {
        // Iterate through all points and data of interest (X,Y,tang,curv)
        // int rows_ = (int)sizeof(parameters);
        std::cout << "started spline data local" << std::endl;
        std::vector<double> u_first_pass;
        std::vector<double> u_second_pass;
        double X_copy[X_SIZE];
        double u_max_temp=0.0;
        double u_forward_temp=X[3];
        double u_backward_temp=X[3];
        std::copy(X, X+X_SIZE, X_copy); 
        PointsData spline_data{lookahead_,4};
        for (long int i{ 0 }; i < lookahead_; i++) { //parameter_writing + first pass
            double param = parameters[i];
            Point temp1 = spline_final->getPoint(param);
            double temp2 = spline_final->getTangent(param);
            double curv = spline_final->getCurvature(param);
            NormalForces Fz = getFz(X_copy);
            MuConstraints mu = getEllipseParams(Fz.Frz);
            u_max_temp = spline_final->getVelocity(param,mu.my_max,v_limit_);
            spline_data(i, 0) = temp1(0);
            spline_data(i, 1) = temp1(1);
            spline_data(i, 2) = temp2;
            // spline_data(i, 3) = u_max_temp;
            if(i==0) u_forward_temp = u_max_temp;
            else {
                X_copy[3] = u_forward_temp;
                double Fy_remain = m*std::pow(X_copy[3],2)*curv;
                double Fx_remain_powered = std::pow(m*g*fr_par,2) - std::pow(Fy_remain,2);
                double Fx_remain;
                if(Fx_remain_powered<=0.0) Fx_remain = 0.0;
                else Fx_remain = std::sqrt(Fx_remain_powered);
                u_forward_temp = std::sqrt(std::pow(X_copy[3],2)+2*(Fx_remain/m)*sol*std::abs(parameters[i]-parameters[i-1]));
                X_copy[3] = u_forward_temp; //for next iteration 
            }
            double u_first_temp = custom_min(u_forward_temp,u_max_temp);
            u_first_pass.push_back(u_first_temp);
        }
        X_copy[3] = u_first_pass[lookahead_-1];
        for (long int j{ 0 }; j < lookahead_; j++) { //second pass
            double param = parameters[lookahead_-1-j];
            double curv = spline_final->getCurvature(param);
            if(j==0) u_backward_temp = u_first_pass[lookahead_-1-j];
            else {
                X_copy[3] = u_backward_temp;
                double Fy_remain = m*std::pow(X_copy[3],2)*curv;
                double Fx_remain_powered = std::pow(m*g*fr_par,2) - std::pow(Fy_remain,2);
                double Fx_remain;
                if(Fx_remain_powered<=0.0) Fx_remain = 0.0;
                else Fx_remain = std::sqrt(Fx_remain_powered);
                double u_backward_powered = std::pow(X_copy[3],2)-2*(Fx_remain/m)*sol*std::abs(parameters[lookahead_-j]-parameters[lookahead_-j-1]);
                if(u_backward_powered<=0.0) u_backward_temp = X_copy[3];
                else u_backward_temp = std::sqrt(u_backward_powered); 
            }
            double u_second_temp = custom_min(u_backward_temp,u_first_pass[lookahead_-1-j]);
            u_second_pass.push_back(u_second_temp);
        }
        std::reverse(u_second_pass.begin(), u_second_pass.end());
        for (long int i{ 0 }; i < lookahead_; i++) {
            if(lap_counter < total_laps_) spline_data(i,3) = u_second_pass[i];
            if(mission_=="skidpad" and (lap_counter<=1 or lap_counter==3)) spline_data(i,3) = 5.0; 
            else if (lap_counter == total_laps_ and X[3]>=6.0) spline_data(i,3) = 6.0;
            else if (lap_counter == total_laps_ and X[3]<6.0 and X[3]>=3.0) spline_data(i,3) = 3.0;
            else if (lap_counter == total_laps_ and X[3]<3.0) {
                brake_flag = 1;
                spline_data(i,3) = 0.0;
            }
            if(emergency==1) spline_data(i,3) = 3.0;  
        }
        std::cout << "values of all passes are: " << u_first_pass[1] << " " << u_second_pass[1] << " " << spline_data(1,3) << std::endl;
        return spline_data;
        std::cout << "got spline data local" << std::endl;
    }

    void MpcSolver::writeParamsKnown(int global_int) {
        std::cout << "started writing of known_params" << std::endl;
        for (int i = 1; i < lookahead_; ++i) {
            if(s_array_final[i]>sol) std::cout << "finished lap" << std::endl;
            if(emergency || global_int==-1 ) s_array_final[i] = s_array_final[i-1] + s_interval_;
            else {
                double step_temp = ds_vector[i-1]*dt;
                if(mission_=="trackdrive") step_temp = 0.15;
                // if(mission_=="skidpad") step_temp = 0.1;
                if(mission_=="skidpad") step_temp = 0.15;
                if(mission_=="autox") step_temp = 0.2;
                if(mission_=="accel") step_temp = 0.2; //stathero
                // s_array_final[i] = s_array_final[i-1] +  s_interval_;
                if(step_temp>s_space_max) step_temp = s_space_max;
                if(step_temp<s_space_min) step_temp = s_space_min;
                s_array_final[i] = s_array_final[i-1] +  step_temp; //changed
            }  
            if(mission_=="skidpad") { 
                if(s_array_final[i]>sol*0.3 and lap_counter==5) s_array_final[i]=sol*0.3;
                if(lap_counter>0 and lap_counter<=4 and s_array_final[i]>sol ) s_array_final[i]=0.0;
                if(lap_counter==0 and s_array_final[i]>sol) s_array_final[i]=sol-1e-6;
            } 
            if(mission_=="accel") { 
                if(s_array_final[i]>sol*0.52 and lap_counter==1) s_array_final[i]=sol*0.52;
                if(lap_counter==0 and s_array_final[i]>sol) s_array_final[i]=sol-1e-6;
            }
            if(mission_=="trackdrive") { 
                if(s_array_final[i]>=sol) s_array_final[i]=0.0;
            }
        }
        for(int i=0; i<lookahead_; ++i) s_array_final[i]=s_array_final[i]/sol;
        std::cout << "finished generation of s_profile" << std::endl;
        params_array = getSplineDataLocal(s_array_final);
        copyToParameters();
    }

    void MpcSolver::writeParamsUnknown() {
        std::cout << "started writing of unknown_params" << std::endl;
        s_array_final[0] = (float)s_array_final[0];
        for (int i = 1; i < lookahead_; ++i) {
            s_array_final[i] = s_array_final[i-1] + s_interval_;
            if(s_array_final[i]>=sol) s_array_final[i]=sol-1e-3;
        }
        for(int i=0; i<lookahead_; ++i) s_array_final[i]=s_array_final[i]/sol;
        std::cout << "finished generation of s_profile" << std::endl;
        std::cout << "s_final params 0,1,5 and 19 are: " << s_array_final[0] << " " << s_array_final[1] << " " << s_array_final[5] << " " << s_array_final[19] << std::endl;
        std::cout << "generated last autox point: " << spline_final->getPoint(s_array_final[19]) << std::endl;
        params_array = getSplineDataLocal(s_array_final);
        copyToParameters();
    }

    void MpcSolver::copyToParameters() {
        for(int k = 0; k < 4*lookahead_; ++k) { 
            int mod_ = k%4;
            if (mod_ == 0) {
                params.all_parameters[k] = params_array(int(k/4),0); 
                if((mission_=="autox" or mission_=="trackdrive") and finish_flag==1) params.all_parameters[k] = start_point.x;
                // std::cout << "added X " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if(mod_ == 1) {
                params.all_parameters[k] = params_array(int((k-1)/4),1);
                if((mission_=="autox" or mission_=="trackdrive") and finish_flag==1) params.all_parameters[k] = start_point.y;
                // std::cout << "added Y " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if(mod_ == 2) {
                params.all_parameters[k] = params_array(int((k-2)/4),2);
                if((mission_=="autox" or mission_=="trackdrive") and finish_flag==1) params.all_parameters[k] = 0.0; //to be changed
                // std::cout << "added phi " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
            else if (mod_ == 3) {
                params.all_parameters[k] = params_array(int((k-3)/4),3);
                // std::cout << "added curv " << params.all_parameters[k] <<  " at kappa  " << k << std::endl;
            }
        }
        std::cout << "finished writing of all unknown_params" << std::endl;
    }

    void MpcSolver::generateTrackConfig() {
        if(mission_=="accel") { 
            start_point.x = -0.3-wing-l_f;
            start_point.y = 0.0;
            center_point.x = 75+wing+l_f+0.3;
            center_point.y = 0.0;
            midpoints_txt_ = "src/6.Controls/mpc/data/Acceleration.txt";
            lookahead_ = 30;
            known_track_ = true; 
        }
        if(mission_=="skidpad") {
            start_point.x = -15 - l_f - wing; 
            start_point.y = 0.0;
            center_point.x = 0.0;
            center_point.y = 0.0;
            midpoints_txt_ = "src/6.Controls/mpc/data/skidpad_straight1.txt";
            lookahead_ = 30;
            known_track_ = true; 
        }
        if(mission_=="autox") {
            start_point.x = -6 - l_f ;
            start_point.y = 0.0;
            center_point.x = 0.0;
            center_point.y = 0.0;
            lookahead_ = 30;
            known_track_ = false; 
        }
        if(mission_=="trackdrive") {
            if(!simulation_){
                start_point.x = -( 6 + l_f); 
                start_point.y = 0.0;
            }
            else {
                start_point.x = 0.0; 
                start_point.y = 6+l_f;
            }
            center_point.x = 0.0;
            center_point.y = 0.0;
            midpoints_txt_ = "src/6.Controls/mpc/data/trackdrive_midpoints_2.txt";
            lookahead_ = 30;
            known_track_ = true; 
        }
    }

    void MpcSolver::updateSkidpadSpline(int lap_counter) {
        if(lap_counter==1 or lap_counter==2) updateSplineParameters("src/6.Controls/mpc/data/skidpad_right.txt");
        if(lap_counter==3 or lap_counter==4) updateSplineParameters("src/6.Controls/mpc/data/skidpad_left.txt");
        if(lap_counter==5) updateSplineParameters("src/6.Controls/mpc/data/skidpad_straight2.txt");
    }

    void MpcSolver::customLapCounter() {
        double critical_dist = std::sqrt(std::pow(X[0] - center_point.x, 2) + std::pow(X[1] - center_point.y, 2));
        if(mission_=="skidpad") {
            if(critical_dist <= 2*l_f+0.5 && lap_lock==0) {
                lap_lock=1;
                lap_counter++;
            }
            if(critical_dist > 2*l_f+0.5) lap_lock=0;
        }
        if(mission_=="accel") {
            if(critical_dist<l_f and lap_lock==0) {
                lap_counter++;
                lap_lock=1;
            }
        }
        if(mission_=="autox") {
            if(critical_dist<l_f and lap_lock==0) {
                lap_counter++;
                lap_lock=1;
            }
            if(critical_dist > l_f) lap_lock=0;
        }
        if(mission_=="trackdrive") {
            if(critical_dist<l_f and lap_lock==0) {
                lap_counter++;
                lap_lock=1;
            }
            if(critical_dist > l_f) lap_lock=0;
        }
    }

    void MpcSolver::generateFinishFlag(int lap_counter) {
        if(mission_=="accel" and lap_counter==1){
            finish_flag=1;
        }
        if(mission_=="skidpad" and lap_counter==5){
            finish_flag=1;
        }
        if(mission_=="autox" and lap_counter==2){
            finish_flag=1;
        }
        if(mission_=="trackdrive" and lap_counter==11){
            finish_flag=1;
        }
    }

    void MpcSolver::updateSplineParameters(std::string txt_file) {
        std::cout << "File I read from is: " << txt_file << std::endl;
        Eigen::MatrixXd spline_input = readTrack(txt_file);
        path_planning::PointsArray midpoints{spline_input};
        midpoints.conservativeResize(midpoints.rows(), midpoints.cols());
        // midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
        path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);
        spline_final = spline_init;
        spline_resolution = int (spline_init->getApproximateLength()/s_interval_);
        sol = spline_init->getApproximateLength(); 
        std::cout << "length of track and resolution are: "<< sol << " " << spline_resolution << std::endl;
        whole_track = spline_init->getSplineData(spline_resolution);
        std::cout << "read known track" << std::endl;
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
        // if(mission_!="autox") {
        //     s_vector.push_back(output.x21[2]);
        //     s_vector.push_back(output.x22[2]);
        //     s_vector.push_back(output.x23[2]);
        //     s_vector.push_back(output.x24[2]);
        //     s_vector.push_back(output.x25[2]);
        //     s_vector.push_back(output.x26[2]);
        //     s_vector.push_back(output.x27[2]);
        //     s_vector.push_back(output.x28[2]);
        //     s_vector.push_back(output.x29[2]);
        //     s_vector.push_back(output.x30[2]);
        // }
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
    }

    void MpcSolver::generateOutput() { 
        //dF ddelta dindex
        if(exitflag==1) for(int k = 0; k<3; k++) U[k]=output.x02[k];
        else for(int k = 0; k<3; k++) U[k]=0.0;
        //writeLookaheadArray1();
        if(mission_!="autox") writeLookaheadArray2();        
        //define message to ROS2
        Integrator();
        if(!finish_flag) steer_last = X[7];
        // if(mission_=="accel" and finish_flag==1) { //just no steering when finishing accel
        //     X[7] = 0.0; 
        //     U[1] = 0.0;
        // }
        if(finish_flag==1 and mission_!="skidpad" ) X[7]=steer_last;
        if(brake_flag==1) { //just no steering when entering braking for all events
            X[7] = 0.0; 
            U[1] = 0.0;
        } 
        checkReliability();
        if(X[6]>2000.0) X[6]=2000.0;
        if(X[6]<-2000.0) X[6]=-2000.0; //to be added as parameter
        if(X[7]>29.5/57.2958) X[7] = 29.5/57.2958;
        if(X[7]<-(29.5/57.2958)) X[7] = -(29.5/57.2958);
        if(!brake_flag) {
            output_struct.speed_target = (int)(5.0);
            output_struct.speed_actual = (int)(vel_struct.velocity_x);
            output_struct.motor_torque_target = (float)(X[6]*Rw/(gr*eff));
            output_struct.steering_angle_target = (float)(X[7]);
            output_struct.brake_pressure_target = (bool)(0);
        }
        else { 
            output_struct.speed_target = (int)(5.0);
            output_struct.speed_actual = (int)(vel_struct.velocity_x);
            output_struct.motor_torque_target = (float)(0.0);
            output_struct.steering_angle_target = (float)(X[7]);
            output_struct.brake_pressure_target = (bool)(1);
        }
        std::cout << "U final array is: " << U[0] << " " << U[1] << " " << U[2] << std::endl;
        std::cout << "solver_s is: " << output.x02[11] << std::endl;
    }

    void MpcSolver::checkReliability() {
        NormalForces Fz = getFz(X);
        MuConstraints mu = getEllipseParams(Fz.Frz);
        SlipAngles sa = getSlipAngles(X);
        double Fry = getFy(Fz.Frz,sa.sar);
        double ellipse_per = std::pow(X[6]/(mu.mx_max*Fz.Frz),2) + std::pow(Fry/(mu.my_max*Fz.Frz),2);
        if(ellipse_per>1.0) {
            std::cout << "Vgika apo ellipse with Frx_bef -> " << X[6] << std::endl;
            double Frx_remain_act = std::pow(Fz.Frz,2) - std::pow(Fry/mu.my_max,2);
            if(Frx_remain_act<=0.0) Frx_remain_act = 0.0;
            if(X[6]<=0.0) X[6] = -0.8*mu.mx_max*std::sqrt(Frx_remain_act);
            else X[6] = 0.8*mu.mx_max*std::sqrt(Frx_remain_act);
            std::cout << "Vgika apo ellipse with Frx_after -> " << X[6] << std::endl;
            ellipse_counter+=1;
        }
        std::cout << "Emergency and ellipse counters are: " << emergency_counter << " " << ellipse_counter << std::endl;
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