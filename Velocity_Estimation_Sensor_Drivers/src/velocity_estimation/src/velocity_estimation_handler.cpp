#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include "velocity_estimation_handler.h"

// TODO: detect when vectornav goes into mode 2: INS Tracking

namespace ns_vel_est
{
VelocityEstimationHandler::VelocityEstimationHandler()
    : Node("velocity_estimation_node"), estimator_{ this } {

    RCLCPP_INFO(this->get_logger(), "Initializing node");
    loadParameters();

    // Initialize the states object and set the time step
    get_parameter("frequency", node_frequency_);
    estimator_.setDeltaTime(1.0 / node_frequency_);
    estimator_.init();

    // Initialize the measurement and update vector with zeros
    measurement_vector_.setZero();
    update_vector_ = { 1, 1, 1, 1, 1, 0, 0, 0, 0 };

    setSubscribers();

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("state_estimation/twist", 10);

    // We dont' care about the lost precision of int division, wall_timer gets int values anyway...
    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / node_frequency_), std::bind(&VelocityEstimationHandler::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Velocity Estimator is Online");
}

// Set node subscribers
void VelocityEstimationHandler::setSubscribers() {
    using std::placeholders::_1;

    // Setting the quality of service for the subscribers to sensor data. This has a smaller buffer
    // for the subscriber and best effort reliability. Go to rmw_qos_profile_sensor_data definition
    // and to ros2 documentation for full details. The KeepLast(5) argument is not used since it gets
    // overwritten afterwards, it is necessary for the QoSInitialization argument.
    auto sensor_qos{ rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data) };

    // TODO: change the topic names according to the new namespaces of each vectornav unit
    vn_velocity_sub_ = create_subscription<vectornav_msgs::msg::InsGroup>("vectornav2/raw/ins",
        sensor_qos, std::bind(&VelocityEstimationHandler::velocityCallback, this, _1)); // vn-300
    // vn_attitude_sub_ = create_subscription<vectornav_msgs::msg::AttitudeGroup>("vectornav2/raw/attitude",
    // sensor_qos, std::bind(&VelocityEstimationHandler::attitudeCallback, this, _1)); // vn-300
    vn_imu_sub_ = create_subscription<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu",
        sensor_qos, std::bind(&VelocityEstimationHandler::imuCallback, this, _1)); // vn-200
}

void VelocityEstimationHandler::publishResults() {
    const StateVector pub_state{ estimator_.getState() };
    const StateMatrix pub_cov{ estimator_.getStateCovariance() };
    geometry_msgs::msg::TwistStamped msg;

    msg.header.stamp = now();
    // TODO: set a valid frame id
    msg.header.frame_id = "car?";

    msg.twist.linear.x = pub_state(StateVx);
    msg.twist.linear.y = pub_state(StateVy);
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = pub_state(StateVyaw);

    pub_->publish(msg);
}
// ROS Callback Functions

void VelocityEstimationHandler::timerCallback() {
    // TODO: timing of the execution and warn if greater than clock cycle
    estimator_.setUpdateVector(update_vector_);
    estimator_.setMeasurements(measurement_vector_);
    update_vector_ = { 1, 1, 1, 1, 1, 0, 0, 0, 0 };
    estimator_.runAlgorithm();
}

void VelocityEstimationHandler::velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg) {
    // Write measurements to corresponding node vector
    measurement_vector_(ObservationVx) = static_cast<double>(msg->velbody.x);
    measurement_vector_(ObservationVy) = static_cast<double>(msg->velbody.y);
    // Set the update vector indices
    update_vector_[ObservationVx] = 1;
    update_vector_[ObservationVy] = 1;
}

// void VelocityEstimationHandler::attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg) {
//     // TODO: see if yaw, pitch, roll measurement could be useful for something
// }

void VelocityEstimationHandler::imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg) {
    // Write measurements to corresponding node vector
    measurement_vector_(ObservationVyaw) = static_cast<double>(msg->angularrate.z);
    measurement_vector_(ObservationAx) = static_cast<double>(msg->accel.x);
    measurement_vector_(ObservationAy) = static_cast<double>(msg->accel.y);
    // Set the update vector indices
    update_vector_[ObservationAx] = 1;
    update_vector_[ObservationAy] = 1;
    update_vector_[ObservationVyaw] = 1;
}

// Loads the node parameters from the .yaml file
void VelocityEstimationHandler::loadParameters() {
    declare_parameter<int>("frequency", 40);

    declare_parameter<std::vector<double>>("initial_state_vector",
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

    declare_parameter<std::vector<double>>("initial_state_covariance",
        { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

    declare_parameter<std::vector<double>>("process_noise_covariance",
        { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

    declare_parameter<std::vector<double>>("measurement_noise_covariance",
        { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

    declare_parameter<double>("vectornav-vn-200.x", 0.0);
    declare_parameter<double>("vectornav-vn-200.y", 0.0);
    declare_parameter<double>("vectornav-vn-300.x", 0.0);
    declare_parameter<double>("vectornav-vn-300.y", 0.0);
}
} // namespace ns_vel_est