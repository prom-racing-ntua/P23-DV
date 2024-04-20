#include "mpc_new.h"

namespace mpc_new{
  new_MpcSolver::new_MpcSolver(){}

  new_MpcSolver::new_MpcSolver(double F_init, int horizonLength, double ds, double dt, double vel_max, double maxF, double minF, std::string mission, int laps){
    this->horizonLength = horizonLength;
    this->dt = dt;
    s_interval_ = ds;
    this->vel_max = vel_max;
    X[6] = F_init;
    F_max = maxF;
    F_min = minF;
    total_laps = laps;
    mem = FORCESNLPsolver_internal_mem(0);
    this->mission = mission;
    if(mission == "skidpad" || mission == "acceleration"){
      Eigen::MatrixXd spline_input;
      if(mission == "skidpad")
        spline_input = readTrack("src/6.Controls/mpc_new/data/skidpad_all.txt");
      else
        spline_input = readTrack("src/6.Controls/mpc_new/data/Acceleration.txt");
      path_planning::PointsArray midpoints{spline_input};
      midpoints.conservativeResize(midpoints.rows(), midpoints.cols());
      path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);
      int spline_resolution = int (spline_init->getApproximateLength()/s_interval_);
      whole_track = spline_init->getSplineData(spline_resolution);
    }
  }
  new_MpcSolver::~new_MpcSolver(){}

  double new_MpcSolver::getFy(double Fz,double sa) {
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

  NormalForces new_MpcSolver::getFz(const double X[X_SIZE]) {
    double a_temp = (X[6]-0.5*CdA*p_air*std::pow(X[3],2))/m;
    double dw = (h_cog/(l_r+l_f))*(a_temp/g)*(m*g);
    const double Ffz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2)) - dw;
    const double Frz = (l_f/(l_f+l_r))*m*g + 0.25*p_air*ClA*(std::pow(X[3],2)) + dw;
    return {Ffz,Frz};
  }

  SlipAngles new_MpcSolver::getSlipAngles(const double X[X_SIZE]) {
    double saf=std::atan((X[4]+l_f*X[5])/(X[3]+1e-3)) - X[7];
    double sar=std::atan((X[4]-l_r*X[5])/(X[3]+1e-3));
    return {saf,sar};
  }

  MuConstraints new_MpcSolver::getEllipseParams(const double &Fz) {
    const double Fz0=1112.0554070627252;
    const double dfz=(Fz-Fz0)/Fz0;
    const double C_tire=0.66;
    const double mx_max=C_tire*(2.21891927-1.36151651e-07*dfz);
    const double my_max=C_tire*(2.46810824-0.21654031*dfz);
    return {mx_max,my_max};
  }

  double new_MpcSolver::convertForceToPressure(float Frx_) {
    double piston_area = 3.14159*std::pow(d_piston/2,2);
    double radius_ratio_r =  (R_disk_r/Rw);
    double pressure_wanted = std::abs(Frx_)/(piston_area*mi_disk*N_rear*radius_ratio_r*100000);
    return pressure_wanted;
  }

  void new_MpcSolver::getF(double X[X_SIZE], double U[U_SIZE], double (&kappa)[X_SIZE]) {
    const double temp=(X[3]-umin)/(umax-umin);
    double double_1 = std::max(temp,0.0);
    double l_a = std::min(double_1,1.0);
    SlipAngles sa = getSlipAngles(X);
    NormalForces Fz = getFz(X);
    double Fry = getFy(Fz.Ffz,sa.sar);
    double Ffy = getFy(Fz.Ffz,sa.saf);
    double Fdrag = 0.5*CdA*p_air*std::pow(X[3],2) + 0.03*(Fz.Frz+Fz.Ffz);
    double beta = std::atan(l_r/(l_f + l_r) * std::tan(X[7]));
    l_a = 1; 

    kappa[0] = (1)*(X[3]*std::cos(X[2]) - X[4]*std::sin(X[2])) + (0)*(X[3]*std::cos(X[2] + beta));  
    kappa[1] = (1)*(X[3]*std::sin(X[2]) + X[4]*std::cos(X[2])) + (0)*(X[3]*std::sin(X[2] + beta));
    kappa[2] = X[5];
    kappa[3] = (1-l_a)*((X[6] - Fdrag)/ m) + (l_a)*((X[6] - Fdrag + Ffy*std::sin(X[7]) + m*X[4]*X[5])/ m); 
    kappa[4] = (1-l_a)*((l_r/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*(((-X[3]*X[5]) + (Fry + Ffy*std::cos(X[7])))/(1.0*m));
    kappa[5] = (1-l_a)*((1/(l_r+l_f))*(kappa[3]*std::tan(X[7])+X[3]*(U[1]/std::pow(std::cos(X[7]),2)))) + (l_a)*((Ffy*l_f*std::cos(X[7]) - Fry*l_r)/(1.0*Iz));
    kappa[6] = U[0];
    kappa[7] = U[1];
  }

  void new_MpcSolver::Integrator() {
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
        X[i] += step_all;
    }
  }

  void new_MpcSolver::build(const double X_data[X_SIZE]){
    //Current situation
    for(int i = 0; i < 6; i++)
      X[i] = X_data[i];
    max_velocity_h = std::max(max_velocity_h, X[3]);
    //Solver initialization
    for (int i = 0; i < X_SIZE; i++) 
      params.xinit[i] = X[i];

    //Storing the first point
    if(global_counter == 0){
      start_point[0] = X[0];
      start_point[1] = X[1];
    }
    
    //Finding my closest point (whole_track(i)->[x, y, phi, curvature])
    double minimum = 100000;
    int index = 0;
    if(mission == "skidpad" || mission == "acceleration"){
      for(int i = 90*points; i < 90*(points+1); i++){
        double distance = std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0));
        if(distance < minimum){
          minimum = distance;
          index = i;
        }
      }
      if(90*(points+1)-index < 10)
        points++;
      if(index > 1700){
        finish = true;
        std::cout << "The maximum velocity so far is: " << max_velocity_h << std::endl;
      }
    }
    else if(mission == "autocross" || mission == "trackdrive"){
      for(int i = 0; i < (int)whole_track.rows(); i++){
        double distance = std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0));
        if(distance < minimum){
          minimum = distance;
          index = i;
        }
      }
      if(global_counter > 10000 && std::sqrt(std::pow((X[0]-start_point[0]),2.0) + std::pow((X[1]-start_point[1]),2.0)) < 0.5)
        finish = true;
    }

    if(minimum > 0.5){
      //Slow down if you are too far 
      if(X[3] >= 1.0){
        output_struct.brake_pressure_target = (double)(10);
        output_struct.motor_torque_target = (float)(-10.0);
      }
      else{ 
        output_struct.brake_pressure_target = (double)(0);
        output_struct.motor_torque_target = (float)(0.0);
      }
      output_struct.speed_target = (float)std::min(X[3], 1.0);
      output_struct.speed_actual = (float)std::min(X[3], 1.0);
      
      output_struct.steering_angle_target = (float)(X[7]);
      if(is_out_of_map){
        //if you are out of track turn very slow
        output_struct.speed_target = (float)std::min(X[3], 0.5);
        output_struct.speed_actual = (float)std::min(X[3], 0.5);
      }
      std::cout << "I am " << minimum << " from the spline!" << std::endl;
    }
    else{
      //Exporting the runtime parameters
      std::vector<double> x_data(horizonLength, 0), y_data(horizonLength, 0), phi_data(horizonLength, 0), curv_data(horizonLength, 0);
      for(int i = 0; i < horizonLength; i++){
        int u = std::min(i+index, (int)whole_track.rows());
        x_data[i] = whole_track(u,0);
        y_data[i] = whole_track(u,1);
        phi_data[i] = whole_track(u,2);
        curv_data[i] = whole_track(u,3);
      }
      std::vector<double> v_data = velocity_profile(x_data, y_data, curv_data);

      //Passing the data to the runtime parameters
      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 4; j++){
          switch(j){
            case 0:
              params.all_parameters[4*i+j] = x_data[i];
              break;
            case 1:
              params.all_parameters[4*i+j] = y_data[i];
              break;
            case 2:
              params.all_parameters[4*i+j] = phi_data[i];
              break;
            case 3:
              params.all_parameters[4*i+j] = v_data[i];
          }
        }

      //Initial guess
      for(int i = 0; i < horizonLength*Z_SIZE; i++)
        params.x0[i] = 0;

      //Calling the solver
      exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, NULL, extfunc_eval);
      if (exitflag == 1) {
          printf("\n\nFORCESNLPsolver returned optimal solution at step %d. Exiting.\n", global_counter);

          //Choosing solver output vector
          double ij = X[3];
          global_counter++;
          double q = ij*(0.1+info.solvetime);
          int t = 0;
          if(global_counter < 60){
              t = (int)(q/s_interval_)+7;
              X[7] = 0;
          }   
          else if((global_counter >= 60 && global_counter < 1100) || global_counter >= 1150)
              t = (int)(q/s_interval_)+2;
          else if(global_counter >= 1100 && global_counter < 1150)
              t = (int)(q/s_interval_)+5; 

          double minima = 10000; 
          for(int i = 0; i<whole_track.rows(); ++i) { 
              minima = std::min(std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0)), minima);
          }
          if(minima > 0.6 && !(global_counter >= 1100 && global_counter < 1250)){
              t = 8;
          }
          choose(t);

          //Producing instructions for the inverter
          Integrator();
          check_reliability();

          if(X[6]>F_max) 
            X[6]=F_max; 
          if(X[6]<F_min) 
            X[6]=F_min; 
          if(X[7]>25.0/57.2958) 
            X[7] = 25.0/57.2958;
          if(X[7]<-(25.0/57.2958)) 
            X[7] = -(25.0/57.2958);

          if(!finish){
            output_struct.speed_target = (float)std::min(X[3], vel_max);
            output_struct.speed_actual = std::min(X[3], vel_max);
            output_struct.motor_torque_target = (float)(X[6]*Rw/(gr*eff));
            if(output_struct.speed_actual<1.0) 
              output_struct.motor_torque_target = (float)(22.5);
            else if(output_struct.motor_torque_target>0.0 and output_struct.motor_torque_target<(float)(15.0)) 
              output_struct.motor_torque_target = 2*output_struct.motor_torque_target;
            output_struct.motor_torque_target = (float)std::min((double)output_struct.motor_torque_target,25.0);
          }
          else{
            output_struct.speed_target = 0.0;
            output_struct.speed_actual = 0.0;
            output_struct.motor_torque_target = -25.0;
          }
          output_struct.steering_angle_target = (float)(X[7]);
          if(X[6] >= 0)
            output_struct.brake_pressure_target = (float)(0.0);  
          else{
            double press_out = convertForceToPressure(X[6]);
            output_struct.brake_pressure_target = (double)(press_out);
          }
      }
      else {
          std::cout << "Checking if im also out of track... " << exitflag << std::endl;
          for(int k = 0; k < U_SIZE; k++) 
            U[k]=0.0;
      }

      /*for(int i = 0; i < horizonLength; i++)
        std::cout << params.all_parameters[4*i+3] << " ";
      std::cout << std::endl;*/
    }
  }

  std::vector<double> new_MpcSolver::velocity_profile(const std::vector<double> x, const std::vector<double> y, std::vector<double> curv){
      struct NormalForces z = getFz(X);
      struct MuConstraints s = getEllipseParams(z.Frz);
      
      if(X[3] < 0)
          X[3] = 0;

      double X_copy[X_SIZE];
      for(int i = 0; i < X_SIZE; i++)
        X_copy[i] = X[i];

      //Local variable to check the end state (in autocross)
      double vx_init = X[3]; //Keeping and the actual current velocity, i will need it to brake

      double u_output = 0, Fx_remain, Fy_remain, F; //Just some helpful variables

      int num = horizonLength;
      //Forward integration
      std::vector<double> u_forward_vector(num, 0);
      double u_first, u_forward; //Just some helpful variables
      for(int i = 0; i < num; i++){
        //Pretty much for the first iteration
        if(std::abs(curv[i]) < 0.0001)
            curv[i] = 0.0001;
        u_first = std::sqrt(s.my_max*g/std::abs(curv[i]));
        
        //Upper bound
        if(u_first > vel_max)
            u_first = vel_max;
        
        if(i == 0)
          u_forward = u_first;
        else if(i > 0 && i < num-1){
          X_copy[3] = u_output;
          z = getFz(X_copy);
          s = getEllipseParams(z.Frz);
          Fy_remain = m*std::abs(curv[i])*std::pow(X_copy[3], 2); //centripetal force
          //Limiting the x-force according to the max frictions
          F = std::pow(z.Frz, 2)-std::pow(Fy_remain/s.my_max, 2);
          if(F < 0)
              Fx_remain = 0;
          else
              Fx_remain = s.mx_max*std::sqrt(F);

          //The "forward" part
          double dx = x[i]-x[i-1];
          double dy = y[i]-y[i-1];
          double ds = std::sqrt(std::pow(dx,2)+std::pow(dy,2));

          u_forward = std::sqrt(std::pow(u_output,2)+2*Fx_remain/m*ds);
        }
        //And to sum up the above
        u_output = std::min(u_forward, u_first);
        u_forward_vector[i] = u_output; 
      }

      //Backward integration
      std::vector<double> u_backward_vector(num, 0);
      double u_backward = 0, u;
      u_output = 0;
      for(int i = 0; i < num; i++){
        if(i == 0)
          u_backward = u_forward_vector[num-1];
        else if(i > 0 && i < num-1){
          X_copy[3] = u_output;
          z = getFz(X_copy);
          s = getEllipseParams(z.Frz);
          Fy_remain = m*std::abs(curv[i])*std::pow(X_copy[3], 2); 
          F = std::pow(z.Frz, 2)-std::pow(Fy_remain/s.my_max, 2);
          if(F < 0)
              Fx_remain = 0;
          else
              Fx_remain = s.mx_max*std::sqrt(F);

          //The "backward" part
          double dx = x[num-i-1]-x[num-i-2];
          double dy = y[num-i-1]-y[num-i-2];
          double ds = std::sqrt(std::pow(dx,2)+std::pow(dy,2));

          u = std::pow(u_output, 2)-2*Fx_remain/m*ds;
          if(u < 0)
              u_backward = u_output;
          else
              u_backward = std::sqrt(u);
        }
        //Sum up
        u_output = std::min(u_backward, u_forward_vector[num-i-1]);
        u_backward_vector[i] = u_output;
      }
      //Flipping the backward vector because i worked from the last element to the first one
      std::reverse(u_backward_vector.begin(), u_backward_vector.end());

      //Now taking into account both integrations and any stop flag i might have
      std::vector<double> u_final(num, 0);
      for(int i = 0; i < num; i++){
        //If i should keep going
        if(!(finish))
          u_final[i] = u_backward_vector[i];
        else
          if(vx_init > 15.0)
              u_final[i] = 10.0;
          else 
              u_final[i] = 0.0;
      }

      return u_final;
    }

  void new_MpcSolver::choose(int t){
    double *p = new double[12];
    switch(t){
      case 0:
          p = output.x01;
          break;
      case 1:
          p = output.x02;
          break;
      case 2:
          p = output.x03;
          break;
      case 3:
          p = output.x04;
          break;
      case 4:
          p = output.x05;
          break;
      case 5:
          p = output.x06;
          break;
      case 6:
          p = output.x07;
          break;
      case 7:
          p = output.x08;
          break;
      case 8:
          p = output.x09;
          break;
      case 9:
          p = output.x10;
          break;
      case 10:
          p = output.x11;
          break;
      case 11:
          p = output.x12;
          break; 
      case 12:
          p = output.x13;
          break;
      case 13:
          p = output.x14;
          break;
      case 14:
          p = output.x15;
          break;
      case 15:
          p = output.x16;
          break;
      case 16:
          p = output.x17;
          break;
      case 17:
          p = output.x18;
          break; 
      case 18:
          p = output.x19;
          break;
      case 19:
          p = output.x20;
          break;
      case 20:
          p = output.x21;
          break;
      case 21:
          p = output.x22;
          break;
      case 22:
          p = output.x23;
          break;
      case 23:
          p = output.x24;
          break; 
      case 24:
          p = output.x25;
          break;
      case 25:
          p = output.x26;
          break;
      case 26:
          p = output.x27;
          break;
      case 27:
          p = output.x28;
          break;
      case 28:
          p = output.x29;
          break;
      default:
          p = output.x30;
          break;     
    }
    for(int k = 0; k < U_SIZE; k++) 
        U[k]=p[k];
  }

  bool new_MpcSolver::check_reliability(){
    NormalForces Fz = getFz(X);
    MuConstraints mu = getEllipseParams(Fz.Frz);
    SlipAngles sa = getSlipAngles(X);
    double Fry = getFy(Fz.Frz,sa.sar);
    double ellipse_per = std::pow(X[6]/(mu.mx_max*Fz.Frz),2) + std::pow(Fry/(mu.my_max*Fz.Frz),2);
    if(ellipse_per > 1.0){ 
      double Frx_remain_act = std::pow(Fz.Frz,2) - std::pow(Fry/mu.my_max,2);
      if(Frx_remain_act<=0.0) 
        Frx_remain_act = 0.0;
      if(X[6]<=0.0) 
        X[6] = -0.8*mu.mx_max*std::sqrt(Frx_remain_act);
      else 
        X[6] = 0.8*mu.mx_max*std::sqrt(Frx_remain_act);
      return false;
    }
    return true;
  }

}
