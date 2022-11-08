#include "localization.h"

namespace slam {

Localization::Localization():robot_pose_counter_ (0), landmark_obs_counter_(0)
{
}


Localization::~Localization()
{
}

// Initializes localization
void Localization::init_localization(double relin_thresh=0.1, double relin_skip=10, double dist_thres=1, double T=0.025)
{
  isam2_relinearize_thresh_ = relin_thresh;
  isam2_relinearize_skip_ = relin_skip;
  dist_threshold = dist_thres;
  dt = T;
    
  // Initialize iSAM2
  initialize_isam2();

  // Initialize the factor graph
  initialize_factor_graph();

}

// Initialize iSAM2 with parameters
void Localization::initialize_isam2()
{
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = isam2_relinearize_thresh_;
  parameters.relinearizeSkip = isam2_relinearize_skip_;
  isam2_ = new gtsam::ISAM2(parameters);
}

// Initialize the factor graph
void Localization::initialize_factor_graph()
{

  current_robot_sym_ = gtsam::Symbol('x', robot_pose_counter_);
  robot_pose_counter_ += 1;

  est_robot_pose_ = gtsam::Pose2(0,0,0);
  gtsam::Matrix3 prior_pose_noise_;
  //prior_pose_noise_.setConstant(0);
  prior_pose_noise_ << 1,0,0,
                       0,1,0,
                       0,0,1;

  factor_graph_ = new gtsam::NonlinearFactorGraph();
  factor_graph_->add(gtsam::PriorFactor<gtsam::Pose2>(current_robot_sym_, est_robot_pose_, gtsam::noiseModel::Gaussian::Covariance(prior_pose_noise_))); // add directly to graph
  init_est_.insert(current_robot_sym_, est_robot_pose_);

  pos_var.setConstant(0);
}


// Adds an odometry measurement to iSAM2 and returns the current estimated state
void Localization::add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise_)
{
  gtsam::Symbol next_robot_sym = gtsam::Symbol('x', robot_pose_counter_);
  robot_pose_counter_ += 1;

  // Update the factor graph with the transformation (need to increment the counter too)
  gtsam::Pose2 robot_odometry((odom_Ux*std::cos(est_robot_pose_.theta()) - odom_Uy*std::sin(est_robot_pose_.theta()))*dt, (odom_Ux*std::sin(est_robot_pose_.theta()) + odom_Uy*std::cos(est_robot_pose_.theta()))*dt, odom_omega*dt);
  J_odom << std::cos(est_robot_pose_.theta())*dt, -std::sin(est_robot_pose_.theta())*dt, 0,  - odom_Ux*std::sin(est_robot_pose_.theta())*dt - odom_Uy*std::cos(est_robot_pose_.theta())*dt,
            std::sin(est_robot_pose_.theta())*dt, std::cos(est_robot_pose_.theta())*dt,  0,  odom_Ux*std::cos(est_robot_pose_.theta())*dt - odom_Uy*std::sin(est_robot_pose_.theta())*dt,
            0,                                    0,                                     dt, 0;
  factor_graph_->add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, next_robot_sym, robot_odometry, gtsam::noiseModel::Gaussian::Covariance(J_odom*odom_noise_*J_odom.transpose())));

  current_robot_sym_ = next_robot_sym;
  
  // Calculate position variance 3x3 matrix (x,y,theta)
  gtsam::Matrix6 pos_vel_var;
  pos_vel_var.setConstant(0);
  J_pose << 1, 0, 0, -std::sin(est_robot_pose_.theta())*odom_Ux*dt-std::cos(est_robot_pose_.theta())*odom_Uy*dt, std::cos(est_robot_pose_.theta())*dt, -std::sin(est_robot_pose_.theta())*dt, 
            0, 1, 0, std::cos(est_robot_pose_.theta())*odom_Ux*dt-std::sin(est_robot_pose_.theta())*odom_Uy*dt,  std::sin(est_robot_pose_.theta())*dt, std::cos(est_robot_pose_.theta())*dt, 
            0, 0, 1, 0,                                                                                          0,                                    dt;
  pos_vel_var.block(0, 0, 3, 3) = pos_var;
  pos_vel_var.block(3, 3, 3, 3) = odom_noise_;
  pos_var = J_pose*pos_vel_var*J_pose.transpose();


  double pos_x = est_robot_pose_.x() + (odom_Ux*std::cos(est_robot_pose_.theta()) - odom_Uy*std::sin(est_robot_pose_.theta()))*dt;
  double pos_y = est_robot_pose_.y() + (odom_Ux*std::sin(est_robot_pose_.theta()) + odom_Uy*std::cos(est_robot_pose_.theta()))*dt;
  double pos_theta = est_robot_pose_.theta() + odom_omega*dt;

  // Estimate is global pose
  gtsam::Pose2 est(pos_x, pos_y, pos_theta);
  init_est_.insert(current_robot_sym_, est);
  est_robot_pose_ = est;


}

// Adds/stores a landmark measurement to iSAM2 and returns whether the factor graph can be optimized
void Localization::add_landmark_measurements(std::vector<PerceptionMeasurement> land_rel, bool localization_only_mode)
{

  if (localization_only_mode) {
    int no_of_land = landmark_id_map_.size();

    for (long unsigned int i=0; i<land_rel.size(); i++){

      // Estimate position of observed cone in global map (x, y)
      gtsam::Matrix12 land_pos;
      land_pos << est_robot_pose_.x() + land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), 
                  est_robot_pose_.y() + land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta());


      // Calculate landmark position variance matrix (2x2) (x,y)
      gtsam::Matrix2 land_var;
      gtsam::Matrix5 land_pos_var;
      land_pos_var.setConstant(0);
      J_land << 1, 0, -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), 
                0, 1, land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()),  std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta());
      land_pos_var.block(0, 0, 3, 3) = pos_var;
      land_pos_var.block(3, 3, 2, 2) = land_rel.at(i).land_obs_noise_;
      land_var = J_land*land_pos_var*J_land.transpose(); // variance matrix of landmark's position which takes into account perception+position variance

      // check already known cones for closest observed cone (Mahalanobis distance), best_match is the index of the closest neighbor, if best_match==-1 then it is a phantom cone
      int best_match = -1;
      double curr_best_dist = dist_threshold;
      for (int j=0; j<no_of_land; j++){
        if (landmark_id_map_.at(j).color == land_rel.at(i).color)
        {
          double dist = (landmark_id_map_.at(j).est_pos - land_pos)*land_var.inverse()*(landmark_id_map_.at(j).est_pos - land_pos).transpose();
          if (dist < curr_best_dist) 
          {
            curr_best_dist = dist;
            best_match = j;
          }
        }
      }
      if (best_match > -1) {
        factor_graph_->add(slam::UnaryFactor(current_robot_sym_, land_rel.at(i).range, land_rel.at(i).theta, landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1], gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
      }  
    }
  }
  else {
    int predeclared_land = landmark_id_map_.size();
    
    for (long unsigned int i=0; i<land_rel.size(); i++){
      
      // Estimate position of observed cone in global map (x, y)
      gtsam::Matrix12 land_pos;
      land_pos << est_robot_pose_.x() + land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), 
                  est_robot_pose_.y() + land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta());
      
      // Calculate landmark position variance matrix (2x2) (x,y)
      gtsam::Matrix2 land_var;
      gtsam::Matrix5 land_pos_var;
      land_pos_var.setConstant(0);
      J_land << 1, 0, -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), 
                0, 1, land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()),  std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta());
      land_pos_var.block(0, 0, 3, 3) = pos_var;
      land_pos_var.block(3, 3, 2, 2) = land_rel.at(i).land_obs_noise_;
      land_var = J_land*land_pos_var*J_land.transpose(); // variance matrix of landmark's position which takes into account perception+position variance

      // check already known cones for closest observed cone (Mahalanobis distance), best_match is the index of the closest neighbor, if best_match==-1 then it is a new cone
      int best_match = -1;
      double curr_best_dist = dist_threshold;
      for (int j=0; j<predeclared_land; j++){
        if (landmark_id_map_.at(j).color == land_rel.at(i).color)
        {
          double dist = (landmark_id_map_.at(j).est_pos - land_pos)*land_var.inverse()*(landmark_id_map_.at(j).est_pos - land_pos).transpose();
          if (dist < curr_best_dist) 
          {
            curr_best_dist = dist;
            best_match = j;
          }
        }
      }
      // Check if the landmark has been observed before
      if (best_match > -1)
      {
        
        // If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate of the landmark
        if (landmark_id_map_.at(best_match).verified == false)
        {
          landmark_id_map_.at(best_match).verified = true;
          // Add the previous landmark measurement to the factor graph from the robot pose symbol
          factor_graph_->add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            landmark_id_map_.at(best_match).robot_pose_sym, landmark_id_map_.at(best_match).land_sym, landmark_id_map_.at(best_match).first_theta, landmark_id_map_.at(best_match).first_range, gtsam::noiseModel::Gaussian::Covariance(landmark_id_map_.at(best_match).first_obs_var)));       
          // Add the initial estimate
          init_est_.insert(landmark_id_map_.at(best_match).land_sym, gtsam::Point2(landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1]));
        }

        // Update mean and covariance based on EKF
        if (landmark_id_map_.at(best_match).optimized == false)
        {
          // Innovation Covariance
          gtsam::Matrix2 S = landmark_id_map_.at(best_match).land_var + land_var;

          // Near-optimal Kalman gain
          gtsam::Matrix2 K = landmark_id_map_.at(best_match).land_var*S.inverse();

          // Updated covariance estimate 
          gtsam::Matrix2 new_land_var = landmark_id_map_.at(best_match).land_var - K*landmark_id_map_.at(best_match).land_var;
          landmark_id_map_.at(best_match).land_var = new_land_var;

          // Updated state estimate 
          gtsam::Matrix12 new_est_pos = landmark_id_map_.at(best_match).est_pos.transpose() + K*(land_pos - landmark_id_map_.at(best_match).est_pos).transpose();
          landmark_id_map_.at(best_match).est_pos = new_est_pos;

          // Improve initial estimate
          init_est_.update(landmark_id_map_.at(best_match).land_sym, gtsam::Point2(new_est_pos[0], new_est_pos[1]));
        }

        // Construct the current landmark measurement
        factor_graph_->add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(current_robot_sym_, landmark_id_map_.at(best_match).land_sym, land_rel.at(i).theta, land_rel.at(i).range, gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
      }

      // Case where the landmark has not been observed before
      else
      {
        // Creating the new landmark symbol and putting it in the dictionary
        gtsam::Symbol next_landmark_sym = gtsam::Symbol('l', landmark_obs_counter_);

        // Create the landmark entry 
        Localization::LandmarkInfo landmark_info;
        
        landmark_info.verified = false;
        landmark_info.optimized = false;
        landmark_info.color = land_rel.at(i).color;
        landmark_info.land_sym = next_landmark_sym;
        landmark_info.robot_pose_sym = current_robot_sym_;
        landmark_info.first_range = land_rel.at(i).range;
        landmark_info.first_theta = land_rel.at(i).theta;
        landmark_info.est_pos = land_pos;
        landmark_info.land_var = land_var;
        landmark_info.first_obs_var = land_rel.at(i).land_obs_noise_;

        landmark_id_map_[landmark_obs_counter_] = landmark_info;

        landmark_obs_counter_ += 1;     

      }
    }
  }
}

// Optimizes the factor graph
void Localization::optimize_factor_graph()
{
  // Update iSAM with the new factors
  isam2_->update(*factor_graph_, init_est_);
  // Each call to iSAM2 update(*) performs one iteration of the iterative
  // nonlinear solver. If accuracy is desired at the expense of time,
  // update(*) can be called additional times to perform multiple optimizer
  // iterations every step.
  isam2_->update();

  // Get the current iSAM2 estimate
  est_state_ = isam2_->calculateEstimate();
  // Update the current estimated robot pose
  est_robot_pose_ = est_state_.at<gtsam::Pose2>(current_robot_sym_);

  // Clear the factor graph and values for the next iteration
  factor_graph_->resize(0);
  init_est_.clear();

  int declared_land = landmark_id_map_.size();
  for (int i=0; i<declared_land; i++)
  {
    if (landmark_id_map_.at(i).verified == true)
    {
      landmark_id_map_.at(i).optimized = true;
      landmark_id_map_.at(i).est_pos = est_state_.at<gtsam::Point2>(landmark_id_map_.at(i).land_sym);
    }
  }
  pos_var.setConstant(0);
  est_state_.print();
}

// Returns the estimated robot pose
Localization::Pose2D Localization::get_est_robot_pose()
{
  Localization::Pose2D pose2D;
  pose2D.x = est_robot_pose_.x();
  pose2D.y = est_robot_pose_.y();
  pose2D.theta = est_robot_pose_.theta();

  return pose2D;
}

std::vector<Localization::Cone> Localization::get_est_map(){
  std::vector<Localization::Cone> est_map;
  Localization::Cone cone;
  int no_of_land = landmark_id_map_.size();
  for (int i=0; i<no_of_land; i++){
    cone.color = landmark_id_map_.at(i).color;
    cone.x = landmark_id_map_.at(i).est_pos[0];  
    cone.y = landmark_id_map_.at(i).est_pos[1];  
    est_map.push_back(cone);
  }
  return est_map;
}

}


int main(){
  slam::Localization loca;
  loca.init_localization(0.1, 10, 0.1, 1);

  gtsam::Matrix3 odom_noise;
  odom_noise << 1,0,0,
                0,1,0,
                0,0,1;
  gtsam::Matrix2 obs_noise;
  obs_noise << 1,0,
               0,1;


  for (int i=0; i<5; i++){
    std::vector<slam::Localization::PerceptionMeasurement> measurements;
    slam::Localization::PerceptionMeasurement meas1;
    meas1.color = 0;
    meas1.range = 1;
    meas1.theta = 3.14/2;
    meas1.land_obs_noise_ = obs_noise;
    measurements.push_back(meas1);
    slam::Localization::PerceptionMeasurement meas2;
    meas2.color = 1;
    meas2.range = 1;
    meas2.theta = -3.14/2;    
    meas2.land_obs_noise_ = obs_noise;
    measurements.push_back(meas2);
    slam::Localization::PerceptionMeasurement meas3;
    meas3.color = 0;
    meas3.range = 1.414;
    meas3.theta = 3.14/4;    
    meas3.land_obs_noise_ = obs_noise;
    measurements.push_back(meas3);
    slam::Localization::PerceptionMeasurement meas4;
    meas4.color = 1;
    meas4.range = 1.414;
    meas4.theta = -3.14/4;    
    meas4.land_obs_noise_ = obs_noise;
    measurements.push_back(meas4);
    loca.add_landmark_measurements(measurements, false);
    loca.add_odom_measurement(1, 0, 0,  odom_noise);
  }

  loca.optimize_factor_graph();

  for (int i=5; i<10; i++){
    std::vector<slam::Localization::PerceptionMeasurement> measurements;
    slam::Localization::PerceptionMeasurement meas1;
    meas1.color = 0;
    meas1.range = 1;
    meas1.theta = 3.14/2;
    meas1.land_obs_noise_ = obs_noise;
    measurements.push_back(meas1);
    slam::Localization::PerceptionMeasurement meas2;
    meas2.color = 1;
    meas2.range = 1;
    meas2.theta = -3.14/2;    
    meas2.land_obs_noise_ = obs_noise;
    measurements.push_back(meas2);
    slam::Localization::PerceptionMeasurement meas3;
    meas3.color = 0;
    meas3.range = 1.414;
    meas3.theta = 3.14/4;    
    meas3.land_obs_noise_ = obs_noise;
    measurements.push_back(meas3);
    slam::Localization::PerceptionMeasurement meas4;
    meas4.color = 1;
    meas4.range = 1.414;
    meas4.theta = -3.14/4;    
    meas4.land_obs_noise_ = obs_noise;
    measurements.push_back(meas4);
    loca.add_landmark_measurements(measurements, false);
    loca.add_odom_measurement(1, 0, 0,  odom_noise);
  }

  loca.optimize_factor_graph();

  loca.add_odom_measurement(-10, 0, 0,  odom_noise);

  for (int i=10; i<15; i++){
    std::vector<slam::Localization::PerceptionMeasurement> measurements;
    slam::Localization::PerceptionMeasurement meas1;
    meas1.color = 0;
    meas1.range = 1;
    meas1.theta = 3.14/2;
    meas1.land_obs_noise_ = obs_noise;
    measurements.push_back(meas1);
    slam::Localization::PerceptionMeasurement meas2;
    meas2.color = 1;
    meas2.range = 1;
    meas2.theta = -3.14/2;    
    meas2.land_obs_noise_ = obs_noise;
    measurements.push_back(meas2);
    slam::Localization::PerceptionMeasurement meas3;
    meas3.color = 0;
    meas3.range = 1.414;
    meas3.theta = 3.14/4;    
    meas3.land_obs_noise_ = obs_noise;
    measurements.push_back(meas3);
    slam::Localization::PerceptionMeasurement meas4;
    meas4.color = 1;
    meas4.range = 1.414;
    meas4.theta = -3.14/4;    
    meas4.land_obs_noise_ = obs_noise;
    measurements.push_back(meas4);
    loca.add_landmark_measurements(measurements, true);
    loca.add_odom_measurement(1, 0, 0,  odom_noise);
  }

  loca.optimize_factor_graph();



  // std::vector<slam::Localization::Cone> est_map = loca.get_est_map();
  // int length = est_map.size();
  // for (int i=0; i<length; i++) std::cout << est_map[i].color << std::endl;

  return 0;
}
