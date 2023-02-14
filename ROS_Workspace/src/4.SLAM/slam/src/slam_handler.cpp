#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <pthread.h>
#include "utils.hpp"

#include "custom_msgs/msg/perception2_slam.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "slam_handler.h"
#include "slam.h"

// GTSAM Includes
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace slam_namespace
{

  SLAM_handler::SLAM_handler(): Node("slam_node"), SLAMObject_{ this }
  {

    loadParameters();

    // Initialize First pose Symbol
    previousGlobalIndex = 0;
    currentRobotSymbol = gtsam::Symbol('x',0);
    
    SLAMObject_.initialize_factor_graph(currentRobotSymbol);

    // Initialize Callbackgroup, set timers and subscriptions
    slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);  
    setSubscribers();
    // Initialize Timers for Optimization and Publishing
    OptimizationTimer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SLAM_handler::optimizationCallback, this), slam_callback_group_);
    PublishTimer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SLAM_handler::publishSLAM, this), slam_callback_group_);

    // TODO: setup publisher
  
    // Open Log Files
    int logTime = static_cast<int>(wtime());

    lapCounterFile.open("lapCounterFile.txt", std::ios::app);
    poseLogFile.open("poseLogFile.txt", std::ios::app);
    mapLogFile.open("mapLogFile.txt", std::ios::app);
    velocityLogFile.open("velocityLogFile.txt", std::ios::app);
    perceptionLogFile.open("perceptionLogFile.txt", std::ios::app);
    timeAnalysisLogFile.open("timeAnalysisLogFile.txt", std::ios::app);

    // Lap count material
    laps_completed = -1;
    cooldown = 0;
    cooldown_max = get_parameter("cooldown_max").as_int();

    //Initialize global lock
    if(pthread_spin_init(&globalLock, PTHREAD_PROCESS_SHARED) != 0){
      RCLCPP_INFO(get_logger(), "Global lock initialization failed: exit program");
      exit(1);
    }

  }

  void SLAM_handler::setSubscribers()
  {
    using std::placeholders::_1;
    rclcpp::SubscriptionOptions options;
    options.callback_group = slam_callback_group_;

    perception_subscription_ = create_subscription<custom_msgs::msg::Perception2Slam>(
      "perception2slam_topic", 10, std::bind(&SLAM_handler::perceptionCallback, this, _1), options);

    velocity_estimation_subscription_ = create_subscription<custom_msgs::msg::VelEstimation>(
        "velocity_estimation", 10, std::bind(&SLAM_handler::velocityEstimationCallback, this, _1), options);
  }


  void SLAM_handler::velocityEstimationCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg)
  {
    double addOdomTiming = wtime();
    int globalIndex;
    double velocityX, velocityY, velocityYaw;
    std::array<double,9> varianceArray;
    double* varianceArrayPointer;

    gtsam::Matrix3 varianceMatrix;

    // Get Data from Velocity Estimation Message
    globalIndex = static_cast<int>(msg->counter) + 1;
    // RCLCPP_INFO(get_logger(),"Vel Est %d", globalIndex);
    velocityX = static_cast<double>(msg->u_x);
    velocityY = static_cast<double>(msg->u_y);
    velocityYaw = static_cast<double>(msg->u_yaw);

    varianceArray = static_cast<std::array<double,9>>(msg->var_matrix);
    varianceArrayPointer = varianceArray.data();
    varianceMatrix = Eigen::Map<gtsam::Matrix3>(varianceArrayPointer);

    // Log velocity estimation data
    velocityLogFile << globalIndex << std::endl << velocityX << std::endl << velocityY << std::endl << velocityYaw << std::endl;
    for (auto i: varianceArray)
      velocityLogFile << i << ' ';
    velocityLogFile << std::endl;

    // Import Velocity Estimation data to SLAM class
    gtsam::Symbol nextRobotSymbol = gtsam::Symbol('x', globalIndex);
    
    // Calculate Stride 
    int stride = globalIndex - previousGlobalIndex;
    previousGlobalIndex = globalIndex;

    pthread_spin_lock(&globalLock);
    bool completed_lap = SLAMObject_.add_odom_measurement(velocityX, velocityY, velocityYaw, varianceMatrix, currentRobotSymbol, nextRobotSymbol, stride);

    if (completed_lap && (cooldown==0)){
      laps_completed++;
      cooldown = cooldown_max;
    }
    else if (cooldown>0) {
      cooldown--;
    }
    pthread_spin_unlock(&globalLock);

    lapCounterFile << "Global Index: " <<  globalIndex << " Laps Completed: " << laps_completed << std::endl;
  
    // Update current pose symbol
    currentRobotSymbol = nextRobotSymbol;
    indexToSymbol[globalIndex] = currentRobotSymbol;
    indexToPose[globalIndex] = SLAMObject_.est_robot_pose;

    addOdomTiming = (wtime() - addOdomTiming)*1000.0;
    timeAnalysisLogFile << "Global Index: " <<  globalIndex << " Add Odometry Time: " << addOdomTiming << std::endl;
  }

   void SLAM_handler::perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg)
  { 
    double addLandmarkTiming = wtime();
    int globalIndex;
    std::vector<int> classesList;
    std::vector<float> thetaList, rangeList;

    // Get Data from Perception Message
    globalIndex = static_cast<int>(msg->global_index) + 1;
    RCLCPP_INFO(get_logger(),"Perception %d", globalIndex);
    classesList = static_cast<std::vector<int>>(msg->class_list);
    thetaList = static_cast<std::vector<float>>(msg->theta_list);
    rangeList = static_cast<std::vector<float>>(msg->range_list);


    pthread_spin_lock(&globalLock);
    if (indexToSymbol.find(globalIndex) != indexToSymbol.end()) //If key is found then add landmark
      SLAMObject_.add_landmark_measurements_slam(classesList, rangeList, thetaList, indexToSymbol[globalIndex], indexToPose[globalIndex]);
    pthread_spin_unlock(&globalLock);

    // Log Perception Data
    perceptionLogFile << globalIndex << std::endl;
    for (auto i: classesList)
      perceptionLogFile << i << ' ';
    perceptionLogFile << std::endl;
    for (auto j: rangeList)
      perceptionLogFile << j << ' ';
    perceptionLogFile << std::endl;
    for (auto k: thetaList)
      perceptionLogFile << k << ' ';
    perceptionLogFile << std::endl;

    addLandmarkTiming = (wtime() - addLandmarkTiming)*1000.0;
    timeAnalysisLogFile << "Global Index: " <<  globalIndex << " Add Landmark Time: " << addLandmarkTiming << std::endl;
  }

  void SLAM_handler::publishSLAM()
  {
    pthread_spin_lock(&globalLock);
    gtsam::Pose2 pub_pose = SLAMObject_.est_robot_pose;
    std::vector<gtsam::Matrix13> pub_map = SLAMObject_.get_est_map();
  
    // Log estimated pose, and estimated map
    poseLogFile << previousGlobalIndex << std::endl;
    pthread_spin_unlock(&globalLock);

    poseLogFile << pub_pose.x() << " " << pub_pose.y() << " " << pub_pose.theta() << std::endl;

    mapLogFile << previousGlobalIndex << std::endl;
    for (long unsigned int i=0; i<pub_map.size(); i++) mapLogFile << pub_map.at(i)[0] << " " << pub_map.at(i)[1] << " " << pub_map.at(i)[2] << std::endl;

  }

  void SLAM_handler::optimizationCallback()
  {
    double optimizationTiming = wtime();
    

    pthread_spin_lock(&globalLock);
    gtsam::NonlinearFactorGraph optimizationFactorGraph {SLAMObject_.factor_graph}; // maybe needs deep-copy
    gtsam::Symbol optimizationRobotSymbol = currentRobotSymbol;
    gtsam::Values optimizationInitEstimates = SLAMObject_.init_est;
    gtsam::Pose2 preOptimizationPose = SLAMObject_.est_robot_pose;

    SLAMObject_.factor_graph.resize(0);
    SLAMObject_.init_est.clear();
    pthread_spin_unlock(&globalLock);

    SLAMObject_.optimize_factor_graph(optimizationFactorGraph, optimizationInitEstimates);
    
    pthread_spin_lock(&globalLock);
    SLAMObject_.impose_optimization(optimizationRobotSymbol, preOptimizationPose);
    pthread_spin_unlock(&globalLock);

    optimizationTiming = (wtime() - optimizationTiming)*1000.0;
    timeAnalysisLogFile << "Optimization Time: " << optimizationTiming << std::endl;

    RCLCPP_INFO(get_logger(),"Just Optimized");
  }


  void SLAM_handler::loadParameters() {
      declare_parameter<double>("small_dist_threshold", 1.8);
      declare_parameter<double>("large_dist_threshold", 2.5);

      declare_parameter<double>("dt", 0.02);

      declare_parameter<std::vector<double>>("left_orange", { 6.0, -3.0});
      declare_parameter<std::vector<double>>("right_orange", { 6.0, 3.0});

      declare_parameter<std::vector<double>>("prior_pose_noise", { 0.03, 0.03, 0.01 });

      declare_parameter<double>("range_limit", 12.0);

      declare_parameter<double>("weight", 0.75);

      declare_parameter<int>("cooldown_max", 10);
  }


} //slam_namespace
