#include "slam.h"
#include "slam_handler.h"


namespace slam_namespace {

UnaryFactor::UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model): gtsam::NoiseModelFactor1<gtsam::Pose2> (model, j), m_range(range), m_theta(theta), cone_x(x), cone_y(y) {}

gtsam::Vector UnaryFactor::evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const
{
      if (H) (*H) = (gtsam::Matrix(2,3)<< (q.x()-cone_x)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), (q.y()-cone_y)/sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), 0,
                                   (q.y()-cone_y)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -pow((q.x()-cone_x),2)/(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)), -1).finished();
      return (gtsam::Vector(2) << sqrt(pow((q.x()-cone_x),2) + pow((q.y()-cone_y),2)) - m_range, std::atan2(cone_y-q.y(), cone_x-q.x()) - q.theta() - m_theta).finished();
}


GraphSLAM::GraphSLAM(Handle nh): node_handler_(nh) {}


// GraphSLAM::~GraphSLAM() {}


// Initializes the factor graph
void GraphSLAM::initialize_factor_graph(gtsam::Symbol start_robot_sym)
{
  small_dist_threshold = node_handler_->get_parameter("small_dist_threshold").as_double();
  large_dist_threshold = node_handler_->get_parameter("large_dist_threshold").as_double();

  dt = node_handler_->get_parameter("dt").as_double();

  auto orange1 = node_handler_->get_parameter("left_orange").as_double_array();
  left_orange << orange1[0], orange1[1];
  auto orange2 = node_handler_->get_parameter("right_orange").as_double_array();
  right_orange << orange2[0], orange2[1];

  auto noise1 = node_handler_->get_parameter("prior_pose_noise").as_double_array();
  prior_pose_noise << noise1[0],0,0,
                      0,noise1[1],0,
                      0,0,noise1[2];

  for (int i=0; i<4; i++) land_obs_counter[i] = 0;

  color_char[0] = 'y';
  color_char[1] = 'b';
  color_char[2] = 's';
  color_char[3] = 'l';

  est_robot_pose = gtsam::Pose2(0,0,0);

  // Start a new factor graph and add the starting point using PriorFactor
  factor_graph.add(gtsam::PriorFactor<gtsam::Pose2>(start_robot_sym, est_robot_pose, gtsam::noiseModel::Gaussian::Covariance(prior_pose_noise))); // add directly to graph
  init_est.insert(start_robot_sym, est_robot_pose);

  weight = node_handler_->get_parameter("weight").as_double();
  range_limit = node_handler_->get_parameter("range_limit").as_double();
}


// Adds an odometry measurement to iSAM2 and returns the current estimated state
bool GraphSLAM::add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise, gtsam::Symbol current_robot_sym, gtsam::Symbol next_robot_sym, int stride)
{
  // Firstly, add the edge between two robot poses based on the velocity estimation received
  gtsam::Pose2 robot_odometry(odom_Ux*stride*dt, odom_Uy*stride*dt, odom_omega*stride*dt);
  gtsam::Matrix3 J_dt;
  J_dt << stride*dt, 0, 0,
          0, stride*dt, 0,
          0, 0, stride*dt;


  factor_graph.add(gtsam::BetweenFactor<gtsam::Pose2> (current_robot_sym, next_robot_sym, robot_odometry, gtsam::noiseModel::Gaussian::Covariance(J_dt*odom_noise*J_dt.transpose())));

  // Secondly, add an initial estimate of the new robot pose on the global map
  gtsam::Pose2 new_pos = gtsam::Pose2(est_robot_pose.x()+(odom_Ux*std::cos(est_robot_pose.theta())-odom_Uy*std::sin(est_robot_pose.theta()))*stride*dt, est_robot_pose.y()+(odom_Ux*std::sin(est_robot_pose.theta())+odom_Uy*std::cos(est_robot_pose.theta()))*stride*dt, est_robot_pose.theta()+odom_omega*stride*dt);
  init_est.insert(next_robot_sym, new_pos);

  gtsam::Matrix12 prev_pose_matrix, curr_pose_matrix;
  prev_pose_matrix << est_robot_pose.x(), est_robot_pose.y();
  curr_pose_matrix << new_pos.x(), new_pos.y();

  bool completed_lap = intersect(prev_pose_matrix, curr_pose_matrix);

  est_robot_pose = new_pos;

  return completed_lap;

}


// void GraphSLAM::add_landmark_measurements_loc(std::vector<PerceptionMeasurement> land_rel){
//   // Find best match for every cone observation arriving from perception
//   for (long unsigned int i=0; i<land_rel.size(); i++){
//     int best_match = findNN(land_rel.at(i));
//     if (best_match > -1) {
//       factor_graph->add(slam::UnaryFactor(current_robot_sym, land_rel.at(i).range, land_rel.at(i).theta, landmark_id_map.at(best_match).est_pos[0], landmark_id_map.at(best_match).est_pos[1], gtsam::noiseModel::Gaussian::Covariance(land_rel.at(i).land_obs_noise_)));
//     }
//     // else: phantom cone
//   }
// }


void GraphSLAM::add_landmark_measurements_slam(std::vector<int> color_list, std::vector<float> range_list, std::vector<float> theta_list, gtsam::Symbol current_robot_sym, gtsam::Pose2 current_pose)
{
  for (long unsigned int i=0; i<color_list.size(); i++){
    int color = color_list.at(i);
    if (range_list.at(i)<range_limit){
      // Estimate position of newly observed cone in global map (x,y)
      gtsam::Matrix12 obs_pos;
      obs_pos << current_pose.x() + range_list.at(i)*std::cos(theta_list.at(i)+current_pose.theta()),
                 current_pose.y() + range_list.at(i)*std::sin(theta_list.at(i)+current_pose.theta());

      gtsam::Matrix2 land_obs_noise;
      if (color == 3){
        land_obs_noise << 0.01,0,
                          0,0.06*range_list.at(i)/10;
      }
      else {
        land_obs_noise << 0.001,0,
                          0,0.02*range_list.at(i)/10;
      }

      int best_match = findNN(color, range_list.at(i), theta_list.at(i), current_pose);
      if (best_match > -1)
      {
        // If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate of the landmark's position (x,y)
        if (landmark_id_map[color].at(best_match).verified == false)
        {
          // Seen for a second time = verified
          landmark_id_map[color].at(best_match).verified = true;

          // Add the previous landmark measurement to the factor graph from the robot pose symbol
          factor_graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(landmark_id_map[color].at(best_match).robot_pose_sym, landmark_id_map[color].at(best_match).land_sym, landmark_id_map[color].at(best_match).first_theta, landmark_id_map[color].at(best_match).first_range, gtsam::noiseModel::Gaussian::Covariance(landmark_id_map[color].at(best_match).first_obs_var)));

          // Add the initial estimate
          init_est.insert(landmark_id_map[color].at(best_match).land_sym, gtsam::Point2(landmark_id_map[color].at(best_match).est_pos[0], landmark_id_map[color].at(best_match).est_pos[1]));

        }

        // Construct the current landmark measurement
        factor_graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(current_robot_sym, landmark_id_map[color].at(best_match).land_sym, theta_list.at(i), range_list.at(i), gtsam::noiseModel::Gaussian::Covariance(land_obs_noise)));
        
        // Correct position of cone every new observation    
        landmark_id_map[color].at(best_match).est_pos = weight*landmark_id_map[color].at(best_match).est_pos + (1-weight)*obs_pos;

      }

      // Case where the landmark has not been observed before
      else
      {
        // Creating the new landmark symbol and putting it in the dictionary
        gtsam::Symbol next_landmark_sym = gtsam::Symbol(color_char[color], land_obs_counter[color]);

        // Create the landmark entry
        LandmarkInfo landmark_info;

        landmark_info.verified = false;

        landmark_info.land_sym = next_landmark_sym;
        landmark_info.est_pos = obs_pos;

        landmark_info.robot_pose_sym = current_robot_sym;
        landmark_info.first_range = range_list.at(i);
        landmark_info.first_theta = theta_list.at(i);
        landmark_info.first_obs_var = land_obs_noise;

        landmark_id_map[color][land_obs_counter[color]++] = landmark_info;
      }
    }
  }
}


int GraphSLAM::findNN(int color, float range, float theta, gtsam::Pose2 current_pose) {
  // Check already known cones for closest observed cone (Mahalanobis distance), best_match is the index of the closest neighbor, if best_match==-1 then it is a phantom cone
  int best_match = -1;
  double curr_best_dist;
  if (color == 3) curr_best_dist = large_dist_threshold;
  else curr_best_dist = small_dist_threshold;
  gtsam::Matrix12 obs_pos;
  obs_pos << current_pose.x() + range*std::cos(theta+current_pose.theta()),
             current_pose.y() + range*std::sin(theta+current_pose.theta());

  for (long unsigned int i=0; i<landmark_id_map[color].size(); i++){
    gtsam::Matrix12 exp_pos = landmark_id_map[color].at(i).est_pos;
    double dist = sqrt((obs_pos - exp_pos)*(obs_pos - exp_pos).transpose());
    if (dist < curr_best_dist)
    {
      curr_best_dist = dist;
      best_match = i;
    }
  }
  return best_match;
}

// Source: https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
bool GraphSLAM::intersect(gtsam::Matrix12 old_pose, gtsam::Matrix12 new_pose){
  return (ccw(old_pose,left_orange,right_orange) != ccw(new_pose,left_orange,right_orange)) && (ccw(old_pose,new_pose,left_orange) != ccw(old_pose,new_pose,right_orange));
}


bool GraphSLAM::ccw(gtsam::Matrix12 A, gtsam::Matrix12 B, gtsam::Matrix12 C){
  return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0]);
}


// Optimizes the factor graph
void GraphSLAM::optimize_factor_graph(gtsam::NonlinearFactorGraph* opt_factor_graph, gtsam::Values opt_init_est)
{

  // Update the current estimated robot pose
  isam2.update(*opt_factor_graph, opt_init_est);
  est_state = isam2.calculateEstimate();

}


void GraphSLAM::impose_optimization(gtsam::Symbol opt_robot_sym, gtsam::Pose2 pre_opt_pose)
{
  // Update the current estimated robot pose
  double new_x = est_state.at<gtsam::Pose2>(opt_robot_sym).x() + std::cos(est_state.at<gtsam::Pose2>(opt_robot_sym).theta() - pre_opt_pose.theta())*(est_robot_pose.x() - pre_opt_pose.x()) - std::sin(est_state.at<gtsam::Pose2>(opt_robot_sym).theta() - pre_opt_pose.theta())*(est_robot_pose.y() - pre_opt_pose.y());
  double new_y = est_state.at<gtsam::Pose2>(opt_robot_sym).y() + std::sin(est_state.at<gtsam::Pose2>(opt_robot_sym).theta() - pre_opt_pose.theta())*(est_robot_pose.x() - pre_opt_pose.x()) + std::cos(est_state.at<gtsam::Pose2>(opt_robot_sym).theta() - pre_opt_pose.theta())*(est_robot_pose.y() - pre_opt_pose.y());
  double new_theta = est_state.at<gtsam::Pose2>(opt_robot_sym).theta() + est_robot_pose.theta() - pre_opt_pose.theta(); 
  est_robot_pose = gtsam::Pose2(new_x, new_y, new_theta);

  // For every landmark that has been optimized update estimated position
  for (int i=0; i<4; i++){
    for (long unsigned int j=0; j<landmark_id_map[i].size(); j++)
    {
      if (est_state.exists<gtsam::Point2>(landmark_id_map[i].at(j).land_sym))
      {
        landmark_id_map[i].at(j).est_pos = est_state.at<gtsam::Point2>(landmark_id_map[i].at(j).land_sym);
      }
    }
  }
}


std::vector<gtsam::Matrix13> GraphSLAM::get_est_map(){
  std::vector<gtsam::Matrix13> est_map;
  for (int i=0; i<4; i++){
    for (long unsigned int j=0; j<landmark_id_map[i].size(); j++){
      gtsam::Matrix13 cone;
      if (landmark_id_map[i].at(j).verified) {
        cone << i, landmark_id_map[i].at(j).est_pos[0], landmark_id_map[i].at(j).est_pos[1];
        est_map.push_back(cone);
      }
    }
  }
  return est_map;
}

}//slam_namespace namesapce