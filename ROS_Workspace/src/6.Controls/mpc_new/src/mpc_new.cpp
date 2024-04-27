#include "mpc_new.h"

namespace mpc_new{
  //Default constructor: need it when building the class which handles the ROS part
  new_MpcSolver::new_MpcSolver(){}

  new_MpcSolver::new_MpcSolver(double F_init, int horizonLength, double ds, double dt, double vel_max, double maxF, double minF, std::string mission,
    double time_delay, double T_max, double T_min, double angle_max, double angle_min, double wb, double wd_front, double CdA,
    double ClA, double p_air, double h_cog, double gr, double Rw, double m, double g, double Iz, int N_rear, double d_piston,
    double R_disk_f, double R_disk_r, double mi_disk, bool dynamic_ds){

    //Initializing everything i need
    this->horizonLength = horizonLength;
    this->dt = dt;
    s_interval_ = ds;
    this->vel_max = vel_max;
    X[6] = F_init;
    F_max = maxF;
    F_min = minF;
    this->mission = mission;
    mem = FORCESNLPsolver_internal_mem(0); //Solver's memory allocation
    std::vector<double> aux(Z_SIZE, 0);
    prevx0 = std::vector<std::vector<double>>(horizonLength, aux);
    this->delay = time_delay;
    this->T_max = T_max;
    this->T_min = T_min;
    this->angle_max = angle_max;
    this->angle_min = angle_min;
    this->wb = wb;
    this->wd_front = wd_front;
    this->CdA = CdA;
    this->ClA = ClA;
    this->p_air = p_air;
    this->h_cog = h_cog;
    this->gr = gr;
    this->Rw = Rw;
    this->m = m;
    this->g = g;
    this->Iz = Iz;
    this->N_rear = N_rear;
    this->d_piston = d_piston;
    this->R_disk_f = R_disk_f;
    this->R_disk_r = R_disk_r;
    this->mi_disk = mi_disk;
    this->dynamic_ds = dynamic_ds;
    l_f = wb*(1-wd_front);
    l_r = wb*wd_front;

    //Load the known track for the respective mission
    if(mission == "skidpad" || mission == "acceleration"){
      this->dynamic_ds = false; //No dynamic runtime parameters in these (symmetric track)
      
      //Extracting the midpoints
      Eigen::MatrixXd spline_input;
      if(mission == "skidpad")
        spline_input = readTrack("src/6.Controls/mpc_new/data/skidpad_all.txt");
      else
        spline_input = readTrack("src/6.Controls/mpc_new/data/Acceleration.txt");
      path_planning::PointsArray midpoints{spline_input};
      midpoints.conservativeResize(midpoints.rows(), midpoints.cols());

      //Building the spline and populating with more points
      path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);
      int spline_resolution = int (spline_init->getApproximateLength()/s_interval_);
      whole_track = spline_init->getSplineData(spline_resolution);
    }
  }
  new_MpcSolver::~new_MpcSolver(){}

  //Check the header file (VD function, dont care about how the parameters were extracted)
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

  //Check the header file (VD function, dont care about how the parameters were extracted)
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

  //Check the header file (VD function, dont care about how the parameters were extracted)
  MuConstraints new_MpcSolver::getEllipseParams(const double &Fz) {
    const double Fz0=1112.0554070627252;
    const double dfz=(Fz-Fz0)/Fz0;
    const double C_tire=0.66;
    const double mx_max=C_tire*(2.21891927-1.36151651e-07*dfz);
    const double my_max=C_tire*(2.46810824-0.21654031*dfz);
    return {mx_max,my_max};
  }

  //Check the header file (VD function, dont care about how the parameters were extracted)
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

  //RK4 integration->https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
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

  //Main function
  void new_MpcSolver::build(const double X_data[X_SIZE]){
    //Current situation
    for(int i = 0; i < 6; i++)
      X[i] = X_data[i];

    //Solver initialization
    for (int i = 0; i < X_SIZE; i++) 
      params.xinit[i] = X[i];

    //Finding my closest point (whole_track(i)->[x, y, phi, curvature])
    double minimum = 100000;
    int index = 0;
    //If the track is known, the whole spline is known, so search every time in a span of 90 points 
    //to find the one closest to me and every time i am reaching the end of the window, move the window forward
    if(mission == "skidpad" || mission == "acceleration"){
      //Find closest distance
      for(int i = 90*points; i < 90*(points+1); i++){
        double distance = std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0));
        if(distance < minimum){
          minimum = distance;
          index = i;
        }
      }
      //Move window
      if(90*(points+1)-index < 10)
        points++;
    }
    else{
      //In case the spline is just from some points (built from the path planning)
      //check all these points and find the closest one to me
      //Find closest distance
      for(int i = 0; i < (int)whole_track.rows(); i++){
        double distance = std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0));
        if(distance < minimum){
          minimum = distance;
          index = i;
        }
      }
    }

    //The lap i am in is given to me from the pose message (and/or the path planning)
    //Below we see in every case when should i start breaking
    if(mission == "skidpad"){
      if(total_laps == 5){
        finish = true;
        break_distance += X[3]*dt;
      }
    }
    else if(mission == "acceleration"){
      if(total_laps == 1)
        finish = true;
    }
    else if(mission == "autocross"){
      if(total_laps == 2)
        finish = true;
    }
    else if(mission == "trackdrive"){
      if(total_laps == 11)
        finish = true;
    }

    //Checking if i am too far from the spline
    if(minimum > 2.0){
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
      //You can change ds dynamically if you want to
      std::vector<double> dynamic_all_s;
      if(X[3] >= 1.0 && dynamic_ds){
        dynamic_all_s = dynamic(x_data, y_data, curv_data, v_data, whole_track(index, 4));
      }

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
      for(int i = 0; i < horizonLength; i++){
        for(int j = 0; j < Z_SIZE; j++)
          if(exitflag == 1 || !(exitflag!=1 && counting_errors >= horizonLength))
            params.x0[i*Z_SIZE+j] = prevx0[i][j];
          else{
            //No previous solutions available, reinitialize the problem
            counting_errors = 0;
            params.x0[i] = 0.0;
          }
      }

      //Calling the solver
      exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, NULL, extfunc_eval);
      //Choosing solver output vector
      double ij = X[3];
      global_counter++;
      //My system has a delay, so from the time i define my problem until the problem is solved, the time needed for the command to 
      //reach the inverter is (delay_time+solve_time) and from the velocity i had when the problem was formulated, i know which output vector to choose
      //In every case i use a small offset, to be a little be after in time
      double q = ij*(delay+info.solvetime);
      int t;
      //Dynamic is used for a certain velocity and upper
      if(!(X[3] >= 1.0 && dynamic_ds))
        t = (int)(q/s_interval_)+2;
      else{
        int w = 0;
        //Since ds changes, i have to see which vector corresponds to the distance computed above exhaustively
        while(dynamic_all_s[w] < q && w < horizonLength)
          w++;
        t = w+2;
      }
      if (exitflag == 1) {
          printf("\n\nFORCESNLPsolver returned optimal solution at step %d. Exiting.\n", global_counter);

          //In the beginning dont turn at all (bad behaviour in the beginning)
          if(global_counter < 60){
              t = (int)(q/s_interval_)+7;
              X[7] = 0;
          }   

          //If you are away from the spline, use a far vector because vectors which correspond to bigger distance avoid oscillations
          double minima = 10000; 
          for(int i = 0; i<whole_track.rows(); ++i) { 
              minima = std::min(std::sqrt(std::pow((X[0]-whole_track(i,0)),2.0) + std::pow((X[1] - whole_track(i,1)),2.0)), minima);
          }
          if(minima > 0.6){
              t = 8;
          }
          counting_errors = 0; //Everything ok, assign to 0
          choose_and_assign(t, exitflag); 
      }
      else {
        std::cout << "Checking if im also out of track... " << exitflag << std::endl;
        counting_errors += t;
        //If i have available previous solutions
        if(counting_errors < horizonLength){
          choose_and_assign(counting_errors, exitflag);
        }
        else{
          //No available previous solutions, problem must be formulated from the beginning
          for(int k = 0; k < U_SIZE; k++) 
            U[k]=0.0;
          std::vector<double> aux(Z_SIZE, 0);
          prevx0 = std::vector<std::vector<double>>(horizonLength, aux);
        }
      }
      //Produce the next predicted state using RK4 integrator
      Integrator();
      //Checking if this is reliable
      check_reliability();
      //Producing the proper output
      produce_output();
    }
  }

  std::vector<double> new_MpcSolver::velocity_profile(const std::vector<double> x, const std::vector<double> y, const std::vector<double> curv){
      struct NormalForces z = getFz(X);
      struct MuConstraints s = getEllipseParams(z.Frz);
      
      if(X[3] < 0)
          X[3] = 0;

      double X_copy[X_SIZE];
      for(int i = 0; i < X_SIZE; i++)
        X_copy[i] = X[i];

      //Local variable to check the end state (in autocross)
      double vx_init = X[3]; //Keeping and the actual current velocity, i will need it to brake

      double u_output = 0, Fx_remain, Fy_remain, F, accel, max_accel_eng; //Just some helpful variables

      int num = horizonLength;
      //Forward integration
      std::vector<double> u_forward_vector(num, 0);
      double u_first = 100, u_forward; //Just some helpful variables
      for(int i = 0; i < num; i++){
        //Pretty much for the first iteration
        if(std::abs(curv[i]) != 0.0)
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

          max_accel_eng = safety_factor*(F_max-0.5*CdA*p_air*std::pow(X[3],2)-0.1*m*g)/m;
          accel = std::min(Fx_remain/m, max_accel_eng)*safety_factor;
          u_forward = std::sqrt(std::pow(u_output,2)+2*accel*ds);
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

          max_accel_eng = safety_factor*(F_max-0.5*CdA*p_air*std::pow(X[3],2)-0.1*m*g)/m;
          accel = std::min(Fx_remain/m, max_accel_eng)*safety_factor;
          u = std::pow(u_output, 2)-2*accel*ds;
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
        else{
          double decel = std::abs(F_min/m);
          u_final[i] = std::max(vx_init-decel*dt, 0.0);
        }
      }

      return u_final;
    }

  void new_MpcSolver::choose_and_assign(int t, int u){
    //Assignment part
    if(u == 1){
      double *p = new double[Z_SIZE];
      for(int i = 0; i < horizonLength; i++){
        switch(i+t){
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
        for(int j = 0; j < Z_SIZE; j++)
          prevx0[i][j] = p[j];
      }
    }
    else{
      std::vector<double> aux(Z_SIZE, 0);
      std::vector<std::vector<double>> aux_prevx0(horizonLength-counting_errors, aux);
      //Sliding by t on the previous used output
      for(int i = 0; i < horizonLength-counting_errors; i++)
        for(int j = 0; j < Z_SIZE; j++)
          aux_prevx0[i][j] = prevx0[std::min(i+t, horizonLength-1)][j];
      //Defining the next slided output
      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < Z_SIZE; j++)
          prevx0[i][j] = aux_prevx0[std::min(i, horizonLength-counting_errors-1)][j];
    }
    //Choose part
    for(int k = 0; k < U_SIZE; k++) 
      U[k]=prevx0[0][k];
  }

  //Pretty much just checking the ellipses parameters and in case of an emergency just doing proper assignments
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

  void new_MpcSolver::produce_output(){
    //Clip the output to be within the boundaries i demand
    if(X[6]>F_max) 
    X[6]=F_max; 
    if(X[6]<F_min) 
      X[6]=F_min; 
    if(X[7]>angle_max) 
      X[7] = angle_max;
    if(X[7]<angle_min) 
      X[7] = angle_min;

    //In case we dont have to stop
    if(!finish){
      //Clipping the output
      output_struct.speed_target = (float)std::min(X[3], vel_max);
      output_struct.speed_actual = std::min(X[3], vel_max);
      //Convert force to torque
      output_struct.motor_torque_target = (float)(X[6]*Rw/(gr*eff));
      //Torque must not be very low when the velocity is small
      if(output_struct.speed_actual<1.0) 
        output_struct.motor_torque_target = (float)(22.5);
      else if(output_struct.motor_torque_target>0.0 and output_struct.motor_torque_target<(float)(15.0)) 
        output_struct.motor_torque_target = 2*output_struct.motor_torque_target;
      //Clipping the torque
      output_struct.motor_torque_target = (float)std::min((double)output_struct.motor_torque_target,T_max);
    }
    else if(finish){
      //If we have to finish, just start slowing down smoothly
      if(mission == "skidpad" && break_distance > 30.0){
        double decel = std::abs(F_min/m);
        output_struct.speed_target = (float)std::max(std::min(X[3]-decel*dt, vel_max), 0.0);
        output_struct.speed_actual = (float)std::max(std::min(X[3]-decel*dt, vel_max), 0.0);
        output_struct.motor_torque_target = T_min;
      }
      else{
        double decel = std::abs(F_min/m);
        output_struct.speed_target = (float)std::max(std::min(X[3]-decel*dt, vel_max), 0.0);
        output_struct.speed_actual = (float)std::max(std::min(X[3]-decel*dt, vel_max), 0.0);
        output_struct.motor_torque_target = T_min;
      }
    }

    //Steering angle output
    output_struct.steering_angle_target = (float)(X[7]);
    //Checking if we have to break or not
    if(X[6] >= 0)
      output_struct.brake_pressure_target = (float)(0.0);  
    else if(finish){
      //Enable this only when we have to stop (generally use regen)
      double press_out = convertForceToPressure(X[6]);
      output_struct.brake_pressure_target = (double)(press_out);
    }
  }

  std::vector<double> new_MpcSolver::dynamic(std::vector<double> &x_data, std::vector<double> &y_data, std::vector<double> &phi_data, std::vector<double> &v_data, double current_s){
    //First build the velocity spline
    int rows = v_data.size(), cols = 2;
    Eigen::MatrixXd v_result{ rows, cols };
    for (int i{ 0 }; i < rows; i++){
      v_result(i, 1) = v_data[i];
      v_result(i, 0) = i+1;
    }
    path_planning::PointsArray midpoints{v_result};
    midpoints.conservativeResize(midpoints.rows(), midpoints.cols());
    path_planning::ArcLengthSpline *v_spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);

    //Now using the velocity profile to extract dynamic ds
    std::vector<double> ds_params(horizonLength, 0);
    double length = std::min(spline->getApproximateLength(), v_spline->getApproximateLength());
    double s_aux = current_s*spline->getApproximateLength(), v_aux = v_data[0];
    int count = 0;
    //Starting from the closest point on the spline(s_aux is the initial progression uppon the spline)
    //and computing the next progression: s_new = s_previous+v*dt, where v is the velocity corresponding 
    //to the point of the spline where i am
    while(s_aux < length && count < horizonLength){
      if(s_aux < v_spline->getApproximateLength())
        v_aux = v_spline->getPoint(s_aux/v_spline->getApproximateLength())(0);
      else 
        break;
      v_data[count] = v_aux;
      //Using 2*dt instead of dt due to better results
      s_aux += 2*dt*v_aux;
      //Stop if you reached the end of the spline
      if(s_aux < length)
        ds_params[count++] = s_aux;
    }
    //Populate if you dont have enough points
    if(count < horizonLength){
      double last = ds_params[count-1];
      while(count < horizonLength)
        ds_params[count++] = last;
    }
    //Finally using the dynamic ds in the ds_params to extract the new runtime parameters
    for(int i = 0; i < horizonLength; i++){
      double u = ds_params[i]/spline->getApproximateLength();
      x_data[i] = spline->getPoint(u)(0);
      y_data[i] = spline->getPoint(u)(1);
      phi_data[i] = spline->getTangent(u);
    }

    return ds_params;
  }

}
