#include "GraphSLAM.h"
#include "filedata.h"
#include <fstream>

namespace slam {

UnaryFactor::UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model): gtsam::NoiseModelFactor1<gtsam::Pose2> (model, j), m_range(range), m_theta(theta), cone_x(x), cone_y(y) {}

gtsam::Vector UnaryFactor::evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const
{
      if (H) (*H) = (gtsam::Matrix(2,3)<< (q.x()-cone_x)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), (q.y()-cone_y)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), 0,
                                   (q.y()-cone_y)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -pow((q.x()-cone_x),2)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -1).finished();
      return (gtsam::Vector(2) << sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)) - m_range, std::atan2(cone_y-q.y(), cone_x-q.x()) - q.theta() - m_theta).finished();
}


GraphSLAM::GraphSLAM(double relinearize_threshold=0.1, double relinearize_skip=10, double dist_thres=0.1, double T=0.02) {
  robot_pose_counter_ = 0;
  landmark_obs_counter_ = 0;
  dist_threshold = dist_thres;
  dt = T;

  // Initializes iSAM2
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = relinearize_threshold;
  parameters.relinearizeSkip = relinearize_skip;
  isam2_ = new gtsam::ISAM2(parameters);

  // Initializes the factor graph
  initialize_factor_graph();
}


GraphSLAM::~GraphSLAM() {}


// Initializes the factor graph
void GraphSLAM::initialize_factor_graph()
{
  current_robot_sym_ = gtsam::Symbol('x', robot_pose_counter_++);

  est_robot_pose_ = gtsam::Pose2(0,0,0);
  gtsam::Matrix3 prior_pose_noise_;
  // TODO: Decide what noise to set as prior pose noise
  prior_pose_noise_ << 0.03,0,0,
                       0,0.03,0,
                       0,0,0.01;


  // Start a new factor graph and add the starting point using PriorFactor
  factor_graph_ = new gtsam::NonlinearFactorGraph();
  factor_graph_->add(gtsam::PriorFactor<gtsam::Pose2>(current_robot_sym_, est_robot_pose_, gtsam::noiseModel::Gaussian::Covariance(prior_pose_noise_))); // add directly to graph
  init_est_.insert(current_robot_sym_, est_robot_pose_);

}


// Adds an odometry measurement to iSAM2 and returns the current estimated state
void GraphSLAM::add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise_)
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

  // Secondly, add an initial estimate of the new robot pose on the global map
  gtsam::Pose2 new_pos = gtsam::Pose2(est_robot_pose_.x()+(odom_Ux*std::cos(est_robot_pose_.theta())-odom_Uy*std::sin(est_robot_pose_.theta()))*dt, est_robot_pose_.y()+(odom_Ux*std::sin(est_robot_pose_.theta())+odom_Uy*std::cos(est_robot_pose_.theta()))*dt, est_robot_pose_.theta()+odom_omega*dt);
  init_est_.insert(current_robot_sym_, new_pos);
  est_robot_pose_ = new_pos;

}


void GraphSLAM::add_landmark_measurements_loc(std::vector<PerceptionMeasurement> land_rel){
  // Find best match for every cone observation arriving from perception
  for (long unsigned int i=0; i<land_rel.size(); i++){
    int best_match = findNN(land_rel.at(i));
    if (best_match > -1) {
      factor_graph_->add(slam::UnaryFactor(current_robot_sym_, land_rel.at(i).range, land_rel.at(i).theta, landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1], gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
    }
    // else: phantom cone
  }
}


void GraphSLAM::add_landmark_measurements_slam(std::vector<PerceptionMeasurement> land_rel)
{
  for (long unsigned int i=0; i<land_rel.size(); i++){

    // Estimate position of newly observed cone in global map (x,y)
    gtsam::Matrix12 obs_pos;
    obs_pos << est_robot_pose_.x() + land_rel.at(i).range*std::cos(land_rel.at(i).theta+est_robot_pose_.theta()),
                 est_robot_pose_.y() + land_rel.at(i).range*std::sin(land_rel.at(i).theta+est_robot_pose_.theta());

    int best_match = findNN(land_rel.at(i));
    if (best_match > -1)
    {
      // If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate of the landmark's position (x,y)
      if (landmark_id_map_.at(best_match).verified == false)
      {
        // Seen for a second time = verified
        landmark_id_map_.at(best_match).verified = true;

        // Add the previous landmark measurement to the factor graph from the robot pose symbol
        factor_graph_->add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(landmark_id_map_.at(best_match).robot_pose_sym, landmark_id_map_.at(best_match).land_sym, landmark_id_map_.at(best_match).first_theta, landmark_id_map_.at(best_match).first_range, gtsam::noiseModel::Gaussian::Covariance(landmark_id_map_.at(best_match).first_obs_var)));

        // Add the initial estimate
        init_est_.insert(landmark_id_map_.at(best_match).land_sym, gtsam::Point2(landmark_id_map_.at(best_match).est_pos[0], landmark_id_map_.at(best_match).est_pos[1]));

      }

      // Construct the current landmark measurement
      factor_graph_->add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(current_robot_sym_, landmark_id_map_.at(best_match).land_sym, land_rel.at(i).theta, land_rel.at(i).range, gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
      
      // Correct position of cone every new observation    
      float w = 0.9; 
      landmark_id_map_.at(best_match).est_pos = w*landmark_id_map_.at(best_match).est_pos + (1-w)*obs_pos;

    }

    // Case where the landmark has not been observed before
    else
    {
      // Creating the new landmark symbol and putting it in the dictionary
      gtsam::Symbol next_landmark_sym = gtsam::Symbol('l', landmark_obs_counter_);

      // Create the landmark entry
      GraphSLAM::LandmarkInfo landmark_info;

      landmark_info.verified = false;

      landmark_info.land_sym = next_landmark_sym;
      landmark_info.color = land_rel.at(i).color;
      landmark_info.est_pos = obs_pos;

      landmark_info.robot_pose_sym = current_robot_sym_;
      landmark_info.first_range = land_rel.at(i).range;
      landmark_info.first_theta = land_rel.at(i).theta;
      landmark_info.first_obs_var = land_rel.at(i).land_obs_noise_;

      landmark_id_map_[landmark_obs_counter_++] = landmark_info;
    }
  }
}


int GraphSLAM::findNN(GraphSLAM::PerceptionMeasurement meas) {
  // Check already known cones for closest observed cone (Mahalanobis distance), best_match is the index of the closest neighbor, if best_match==-1 then it is a phantom cone
  int best_match = -1;
  double curr_best_dist;
  if (meas.color == 3) curr_best_dist = 1.1*dist_threshold;
  else curr_best_dist = dist_threshold;
  gtsam::Matrix12 obs_pos;
  obs_pos << est_robot_pose_.x() + meas.range*std::cos(meas.theta+est_robot_pose_.theta()),
             est_robot_pose_.y() + meas.range*std::sin(meas.theta+est_robot_pose_.theta());

  for (long unsigned int j=0; j<landmark_id_map_.size(); j++){
    if (landmark_id_map_.at(j).color == meas.color)
    {
      gtsam::Matrix12 exp_pos = landmark_id_map_.at(j).est_pos;
      double dist = sqrt((obs_pos - exp_pos)*(obs_pos - exp_pos).transpose());
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
void GraphSLAM::optimize_factor_graph()
{
  // auto start_time = std::chrono::high_resolution_clock::now();
  
  gtsam::NonlinearFactorGraph* opt_factor_graph_ = new gtsam::NonlinearFactorGraph();
  gtsam::Values opt_init_est_;

  // Lock this shit
  *opt_factor_graph_ = *factor_graph_;
  opt_init_est_ = init_est_;
  opt_robot_sym_ = current_robot_sym_;
  pre_opt_pose = est_robot_pose_;
  factor_graph_->resize(0);
  init_est_.clear();

  // Update the current estimated robot pose
  isam2_->update(*opt_factor_graph_, opt_init_est_);
  est_state_ = isam2_->calculateEstimate();

  // auto current_time = std::chrono::high_resolution_clock::now();
  // std::cout << "optimization has been running for " << std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() << " milliseconds" << std::endl;

}


void GraphSLAM::impose_optimization()
{
  // Update the current estimated robot pose
  double new_x = est_state_.at<gtsam::Pose2>(opt_robot_sym_).x() + std::cos(est_state_.at<gtsam::Pose2>(opt_robot_sym_).theta() - pre_opt_pose.theta())*(est_robot_pose_.x() - pre_opt_pose.x()) - std::sin(est_state_.at<gtsam::Pose2>(opt_robot_sym_).theta() - pre_opt_pose.theta())*(est_robot_pose_.y() - pre_opt_pose.y());
  double new_y = est_state_.at<gtsam::Pose2>(opt_robot_sym_).y() + std::sin(est_state_.at<gtsam::Pose2>(opt_robot_sym_).theta() - pre_opt_pose.theta())*(est_robot_pose_.x() - pre_opt_pose.x()) + std::cos(est_state_.at<gtsam::Pose2>(opt_robot_sym_).theta() - pre_opt_pose.theta())*(est_robot_pose_.y() - pre_opt_pose.y());
  double new_theta = est_state_.at<gtsam::Pose2>(opt_robot_sym_).theta() + est_robot_pose_.theta() - pre_opt_pose.theta(); 
  est_robot_pose_ = gtsam::Pose2(new_x, new_y, new_theta);

  // For every landmark that has been optimized update estimated position
  for (long unsigned int i=0; i<landmark_id_map_.size(); i++)
  {
    if (est_state_.exists<gtsam::Point2>(landmark_id_map_.at(i).land_sym))
    {
      landmark_id_map_.at(i).est_pos = est_state_.at<gtsam::Point2>(landmark_id_map_.at(i).land_sym);
    }
  }
}


// Returns the estimated robot pose
gtsam::Matrix13 GraphSLAM::get_est_robot_pose()
{
  gtsam::Matrix13 pose;
  pose << est_robot_pose_.x(), est_robot_pose_.y(), est_robot_pose_.theta();
  return pose;
}


std::vector<gtsam::Matrix13> GraphSLAM::get_est_map(){
  std::vector<gtsam::Matrix13> est_map;
  for (long unsigned int i=0; i<landmark_id_map_.size(); i++){
    gtsam::Matrix13 cone;
    if (landmark_id_map_.at(i).verified) {
      cone << landmark_id_map_.at(i).color, landmark_id_map_.at(i).est_pos[0], landmark_id_map_.at(i).est_pos[1];
      est_map.push_back(cone);
    }
  }
  return est_map;
}


void GraphSLAM::Visualize() {
  std::vector<gtsam::Matrix13> est_map = get_est_map();

  std::ofstream data("plot_data.txt");

  for (long unsigned int i=0; i<est_map.size(); i++) {
    if (est_map[i][0] == 0) data << 146;
    else if (est_map[i][0] == 1) data << 9;
    else if (est_map[i][0] == 3) data << 3;
    else data << 7;
    data << ' ';
    data << est_map[i][1];
    data << ' ';
    data << est_map[i][2];
    data << ' ';
    data << '\n';
  }
  // for (int i=0; i<robot_pose_counter_-1; i++) {
  //   gtsam::Pose2 pos = est_state_.at<gtsam::Pose2>(gtsam::Symbol('x', i));
  //   data << 3;
  //   data << ' ';
  //   data << pos.x();
  //   data << ' ';
  //   data << pos.y();
  //   data << ' ';
  //   data << '\n';
  // }
  data.close();
}


void GraphSLAM::read_measurements() {
  read_file read;
  std::vector<slam::GraphSLAM::PerceptionMeasurement> measurements;

  gtsam::Matrix3 odom_noise;

  int odom_num = 1; //starting from line 1
  int per_num = 1;  //starting from line 1
  bool per_eof = 0; //perception end of file
  bool odom_eof = 0;  //odometry end of file
  int prev_pose = -1;
  int current_pose = 0;

  while (!odom_eof && !per_eof) {

    odom_eof = read.read_odometry(odom_num);
    current_pose = read.odometry->timestamp;

    odom_noise << read.odometry->cov_mat[0][0], read.odometry->cov_mat[0][1], read.odometry->cov_mat[0][2],
                  read.odometry->cov_mat[1][0], read.odometry->cov_mat[1][1], read.odometry->cov_mat[1][2],
                  read.odometry->cov_mat[2][0], read.odometry->cov_mat[2][1], read.odometry->cov_mat[2][2];

    if (prev_pose != current_pose) {
      add_odom_measurement(read.odometry->xvelocity, 0, read.odometry->omega,  odom_noise);
    }

    per_eof = read.read_perception(per_num);

    if (read.perception->timestamp == read.odometry->timestamp) {

      slam::GraphSLAM::PerceptionMeasurement meas;
      meas.color = read.perception->color;
      meas.range = read.perception->range;
      meas.theta = read.perception->theta;
      gtsam::Matrix2 obs_noise;
      if (meas.color == 3) {
        obs_noise << 0.05,0,
                     0,meas.range/10;
      }
      else {
        obs_noise << 0.01,0,
                     0,0.6*meas.range/10;
      }
      meas.land_obs_noise_ = obs_noise;
      if (meas.range < 12) measurements.push_back(meas);

      per_num += 4;
      odom_num -= 5; //stay on current timestamp for all perception measurements on this timestamp
    }
    if (!measurements.empty() && read.perception->timestamp != read.odometry->timestamp) {
      add_landmark_measurements_slam(measurements);
      measurements.clear();
    }
    odom_num += 5;
    int abc = 50;
    if (read.odometry->timestamp%abc == abc-2) {  //optimize every 50 timestamps == 1sec
      optimize_factor_graph();
      impose_optimization();
    }
    // if (current_pose==abc) break;
    prev_pose = current_pose;

  }
  std::cout << "Done!" << std::endl;

}

}//slam namesapce


int main(){
  slam::GraphSLAM loca = slam::GraphSLAM(0.01, 1, 2, 0.02);
  loca.read_measurements();
  loca.Visualize();
}