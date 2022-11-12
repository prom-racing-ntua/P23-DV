#include "localization.h"


namespace slam {

UnaryFactor::UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model): gtsam::NoiseModelFactor1<gtsam::Pose2> (model, j), m_range(range), m_theta(theta), cone_x(x), cone_y(y) {}

gtsam::Vector UnaryFactor::evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const 
{
      if (H) (*H) = (gtsam::Matrix(2,3)<< (q.x()-cone_x)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), (q.y()-cone_y)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), 0, 
                                   (q.y()-cone_y)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -pow((q.x()-cone_x),2)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -1).finished();
      return (gtsam::Vector(2) << sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)) - m_range, std::atan2(cone_y-q.y(), cone_x-q.x()) - q.theta() - m_theta).finished();
}


Localization::Localization():robot_pose_counter_ (0), landmark_obs_counter_(0) {}


Localization::~Localization() {}


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
  current_robot_sym_ = gtsam::Symbol('x', robot_pose_counter_++);

  est_robot_pose_ = gtsam::Pose2(0,0,0);
  gtsam::Matrix3 prior_pose_noise_;
  // TODO: Decide what noise to set as prior pose noise
  prior_pose_noise_ << 1,0,0,
                       0,1,0,
                       0,0,1;

  // Start a new factor graph and add the starting point using PriorFactor
  factor_graph_ = new gtsam::NonlinearFactorGraph();
  factor_graph_->add(gtsam::PriorFactor<gtsam::Pose2>(current_robot_sym_, est_robot_pose_, gtsam::noiseModel::Gaussian::Covariance(prior_pose_noise_))); // add directly to graph
  init_est_.insert(current_robot_sym_, est_robot_pose_);

  // In the beginning we are certain of our pose
  pos_var.setConstant(0);
}


// Adds an odometry measurement to iSAM2 and returns the current estimated state
void Localization::add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise_)
{
  gtsam::Symbol next_robot_sym = gtsam::Symbol('x', robot_pose_counter_++);

  // Firstly, add the edge between two robot poses based on the velocity estimation received
  gtsam::Pose2 robot_odometry(odom_Ux*dt, odom_Uy*dt, odom_omega*dt);
  gtsam::Matrix3 J_dt;
  J_dt << dt, 0, 0,
          0, dt, 0,
          0, 0, dt;
  factor_graph_->add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym_, next_robot_sym, robot_odometry, gtsam::noiseModel::Gaussian::Covariance(J_dt*odom_noise_*J_dt.transpose())));

  current_robot_sym_ = next_robot_sym;
  
  // Secondly, calculate position variance 3x3 matrix (x,y,theta)
  gtsam::Matrix6 pos_vel_var;
  pos_vel_var.setConstant(0);
  gtsam::Matrix36 J_pose;
  J_pose << 1, 0, 0, -std::sin(est_robot_pose_.theta())*odom_Ux*dt-std::cos(est_robot_pose_.theta())*odom_Uy*dt, std::cos(est_robot_pose_.theta())*dt, -std::sin(est_robot_pose_.theta())*dt, 
            0, 1, 0, std::cos(est_robot_pose_.theta())*odom_Ux*dt-std::sin(est_robot_pose_.theta())*odom_Uy*dt,  std::sin(est_robot_pose_.theta())*dt, std::cos(est_robot_pose_.theta())*dt, 
            0, 0, 1, 0,                                                                                          0,                                    dt;
  pos_vel_var.block(0, 0, 3, 3) = pos_var;
  pos_vel_var.block(3, 3, 3, 3) = odom_noise_;
  pos_var = J_pose*pos_vel_var*J_pose.transpose();

  // Thirdly, add an initial estimate of the new robot pose on the global map
  gtsam::Pose2 new_pos = gtsam::Pose2(est_robot_pose_.x()+(odom_Ux*std::cos(est_robot_pose_.theta())-odom_Uy*std::sin(est_robot_pose_.theta()))*dt, est_robot_pose_.y()+(odom_Ux*std::sin(est_robot_pose_.theta())+odom_Uy*std::cos(est_robot_pose_.theta()))*dt, est_robot_pose_.theta()+odom_omega*dt);
  init_est_.insert(current_robot_sym_, new_pos);
  est_robot_pose_ = new_pos;
}


void Localization::add_landmark_measurements_loc(std::vector<PerceptionMeasurement> land_rel){
  int no_of_land = landmark_id_map_.size();

  // Find best match for every cone observation arriving from perception
  for (long unsigned int i=0; i<land_rel.size(); i++){

    // Estimate position of observed cone in global map (x, y)
    gtsam::Matrix12 obs_pos;
    obs_pos << est_robot_pose_.x() + land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), 
                est_robot_pose_.y() + land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta());

    // Calculate landmark position variance matrix (2x2) (x,y)
    gtsam::Matrix2 obs_var;
    gtsam::Matrix5 pos_perc_var;
    pos_perc_var.setConstant(0);
    gtsam::Matrix25 J_land;
    J_land << 1, 0, -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), 
              0, 1, land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()),  std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta());
    pos_perc_var.block(0, 0, 3, 3) = pos_var;
    pos_perc_var.block(3, 3, 2, 2) = land_rel.at(i).land_obs_noise_;
    obs_var = J_land*pos_perc_var*J_land.transpose(); // variance matrix of landmark's position which takes into account perception+position variance

    // Find best matching cone from map
    int best_match = findNN(land_rel.at(i).color, obs_pos, obs_var, no_of_land);

    if (best_match > -1) {
      factor_graph_->add(slam::UnaryFactor(current_robot_sym_, land_rel.at(i).range, land_rel.at(i).theta, landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1], gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
    }  
    // else: phantom cone
  }
}


void Localization::add_landmark_measurements_slam(std::vector<PerceptionMeasurement> land_rel)
{
  // We do not want cones that have just been observed in a picture to be associated to each other, therefore we declare the number of cones that pre-existed on the map beforehand
  int no_of_land = landmark_id_map_.size();
  
  for (long unsigned int i=0; i<land_rel.size(); i++){
    
    // Estimate position of observed cone in global map (x,y)
    gtsam::Matrix12 obs_pos;
    obs_pos << est_robot_pose_.x() + land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), 
                est_robot_pose_.y() + land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta());
    
    // Calculate landmark position variance matrix (2x2) (x,y) which takes into account perception+position variance
    gtsam::Matrix2 obs_var;
    gtsam::Matrix5 pos_perc_var;
    pos_perc_var.setConstant(0);
    gtsam::Matrix25 J_land;
    J_land << 1, 0, -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), std::cos(land_rel.at(i).theta+est_robot_pose_.theta()), -land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), 
              0, 1, land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()),  std::sin(land_rel.at(i).theta+est_robot_pose_.theta()), land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta());
    pos_perc_var.block(0, 0, 3, 3) = pos_var;
    pos_perc_var.block(3, 3, 2, 2) = land_rel.at(i).land_obs_noise_;
    obs_var = J_land*pos_perc_var*J_land.transpose(); 

    int best_match = findNN(land_rel.at(i).color, obs_pos, obs_var, no_of_land);

    if (best_match > -1)
    {
      // If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate of the landmark's position (x,y)
      if (landmark_id_map_.at(best_match).verified == false)
      {
        // Seen for a second time = verified
        landmark_id_map_.at(best_match).verified = true;

        // Add the previous landmark measurement to the factor graph from the robot pose symbol
        factor_graph_->add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
          landmark_id_map_.at(best_match).robot_pose_sym, landmark_id_map_.at(best_match).land_sym, landmark_id_map_.at(best_match).first_theta, landmark_id_map_.at(best_match).first_range, gtsam::noiseModel::Gaussian::Covariance(landmark_id_map_.at(best_match).first_obs_var)));       
        
        // Add the initial estimate
        init_est_.insert(landmark_id_map_.at(best_match).land_sym, gtsam::Point2(landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1]));
      }

      // If the landmark has not yet gone through an optimization step, update the mean and variance of its global position (x,y) using a Kalman filter
      if (landmark_id_map_.at(best_match).optimized == false)
      {
        // Innovation Covariance
        gtsam::Matrix2 S = landmark_id_map_.at(best_match).land_var + obs_var;

        // Near-optimal Kalman gain
        gtsam::Matrix2 K = landmark_id_map_.at(best_match).land_var*S.inverse();

        // Updated covariance estimate 
        gtsam::Matrix2 new_land_var = landmark_id_map_.at(best_match).land_var - K*landmark_id_map_.at(best_match).land_var;
        landmark_id_map_.at(best_match).land_var = new_land_var;

        // Updated state estimate 
        gtsam::Matrix12 new_est_pos = landmark_id_map_.at(best_match).est_pos.transpose() + K*(obs_pos - landmark_id_map_.at(best_match).est_pos).transpose();
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

      landmark_info.land_sym = next_landmark_sym;
      landmark_info.color = land_rel.at(i).color;
      landmark_info.est_pos = obs_pos;
      landmark_info.land_var = obs_var;

      landmark_info.robot_pose_sym = current_robot_sym_;
      landmark_info.first_range = land_rel.at(i).range;
      landmark_info.first_theta = land_rel.at(i).theta;
      landmark_info.first_obs_var = land_rel.at(i).land_obs_noise_;

      landmark_id_map_[landmark_obs_counter_++] = landmark_info;
    }
  }
}


int Localization::findNN(int color, gtsam::Matrix12 obs_pos, gtsam::Matrix2 obs_var, int no_of_land) {
  // Check already known cones for closest observed cone (Mahalanobis distance), best_match is the index of the closest neighbor, if best_match==-1 then it is a phantom cone
  // TODO: If we find a way to compute the variance of the position of a landmark after optimization we can take that into account as well when calculating distances
  int best_match = -1;
  double curr_best_dist = dist_threshold;
  for (int j=0; j<no_of_land; j++){
    if (landmark_id_map_.at(j).color == color)
    {
      double dist = (landmark_id_map_.at(j).est_pos - obs_pos)*obs_var.inverse()*(landmark_id_map_.at(j).est_pos - obs_pos).transpose();
      if (dist < curr_best_dist) 
      {
        curr_best_dist = dist;
        best_match = j;
      }
    }
  }
  return best_match;
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

  // For every landmark that has been verified by a second observation, update estimated position and declare them as optimized (i.e. const position in case of data association)
  int declared_land = landmark_id_map_.size();
  for (int i=0; i<declared_land; i++)
  {
    if (landmark_id_map_.at(i).verified == true)
    {
      landmark_id_map_.at(i).optimized = true;
      landmark_id_map_.at(i).est_pos = est_state_.at<gtsam::Point2>(landmark_id_map_.at(i).land_sym);
    }
  }

  // After optimization positional variance is hard to determine so it is declared to be zero
  pos_var.setConstant(0);

  // Clear the factor graph and values for the next iteration
  factor_graph_->resize(0);
  init_est_.clear();

  // Print state for debugging
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
    loca.add_landmark_measurements_slam(measurements);
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
    loca.add_landmark_measurements_slam(measurements);
    loca.add_odom_measurement(1, 0, 0,  odom_noise);
  }

  loca.optimize_factor_graph();
  return 0;
}