#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include "custom_msgs/msg/vel_estimation.hpp"
#include "velocity_estimation_handler.h"

// TODO: detect when vectornav goes into mode 2: INS Tracking
// TODO: subscribe to master to receive counter

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
    updated_sensors_.fill(false);

    setSubscribers();

    pub_ = create_publisher<custom_msgs::msg::VelEstimation>("state_estimation/vel_est", 10);

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

    vn_velocity_sub_ = create_subscription<vectornav_msgs::msg::InsGroup>("vn_300/raw/ins",
        sensor_qos, std::bind(&VelocityEstimationHandler::velocityCallback, this, _1));
    // vn_attitude_sub_ = create_subscription<vectornav_msgs::msg::AttitudeGroup>("vn_300/raw/attitude",
    // sensor_qos, std::bind(&VelocityEstimationHandler::attitudeCallback, this, _1));
    vn_imu_sub_ = create_subscription<vectornav_msgs::msg::ImuGroup>("vn_200/raw/imu",
        sensor_qos, std::bind(&VelocityEstimationHandler::imuCallback, this, _1));
}

void VelocityEstimationHandler::publishResults() {
    constexpr static std::array<int, 3> outputs{ StateVx, StateVy, StateVyaw };
    static unsigned long count{ 0 };
    const StateVector pub_state{ estimator_.getState() };
    const StateMatrix pub_cov{ estimator_.getStateCovariance() };
    custom_msgs::msg::VelEstimation msg;

    msg.counter = count++; // to be changed to master's counter
    msg.u_x = pub_state(StateVx);
    msg.u_y = pub_state(StateVy);
    msg.u_yaw = pub_state(StateVyaw);
    for (size_t i{ 0 }; i < outputs.size(); ++i)
    {
        for (size_t j{ 0 }; j < outputs.size(); ++j)
        {
            msg.var_matrix[i * outputs.size() + j] = pub_cov(outputs[i], outputs[j]);
        }
    }
    pub_->publish(msg);
}

// ROS Callback Functions

void VelocityEstimationHandler::timerCallback() {
    rclcpp::Time starting_time{ this->now() };
    estimator_.setUpdateVector(updated_sensors_);
    // Check if no new measurements came this time step and use dead reckoning if true
    bool dead_reck{ false };
    if (std::none_of(updated_sensors_.begin(), updated_sensors_.end(), [](bool v) { return v; }))
    {
        dead_reck = true;
        RCLCPP_WARN_STREAM(get_logger(), "No measurements received this time step. Dead-reckoning...");
    }
    estimator_.setMeasurements(measurement_vector_);
    updated_sensors_.fill(false);
    estimator_.runAlgorithm(dead_reck);

    rclcpp::Duration total_time{ this->now() - starting_time };
    RCLCPP_INFO_STREAM(get_logger(), "\n-- Execution Completed --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void VelocityEstimationHandler::velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg) {
    // Write measurements to corresponding node vector
    measurement_vector_(ObservationVx) = static_cast<double>(msg->velbody.x);
    measurement_vector_(ObservationVy) = static_cast<double>(msg->velbody.y);
    // Set the update vector indices
    updated_sensors_[VelocitySensor] = true;
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
    updated_sensors_[Accelerometer] = true;
    updated_sensors_[Gyroscope] = true;
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