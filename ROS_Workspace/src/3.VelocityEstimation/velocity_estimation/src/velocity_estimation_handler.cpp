#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include "velocity_estimation_handler.h"

// TODO: detect when vectornav goes into mode 2: INS Tracking

namespace ns_vel_est
{
VelocityEstimationHandler::VelocityEstimationHandler(): Node("velocity_estimation_node"), estimator_{ this } {
    RCLCPP_INFO(this->get_logger(), "Initializing Velocity Estimation node");
    loadParameters();

    pub_ = create_publisher<custom_msgs::msg::VelEstimation>("velocity_estimation", 10);
    cli_ = create_client<custom_msgs::srv::GetFrequencies>("get_frequencies");

    // Initialize the states object and set the time step
    node_frequency_ = getNodeFrequency();
    if (!node_frequency_)
    {
        rclcpp::shutdown();
        return;
    }
    estimator_.setDeltaTime(1.0 / node_frequency_);
    estimator_.init();

    vn_200_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-200.roll").as_double(),
        get_parameter("vectornav-vn-200.pitch").as_double());

    vn_300_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-300.roll").as_double(),
        get_parameter("vectornav-vn-300.pitch").as_double());

    // Initialize the measurement and update vector with zeros
    measurement_vector_.setZero();
    updated_sensors_.fill(false);
    setSubscribers();

    // We dont' care about the lost precision of int division, wall_timer gets int values anyway...
    // timer_ = create_wall_timer(std::chrono::milliseconds(1000 / node_frequency_), std::bind(&VelocityEstimationHandler::timerCallback, this));

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
    //
    // vn_attitude_sub_ = create_subscription<vectornav_msgs::msg::AttitudeGroup>("vn_300/raw/attitude",
    // sensor_qos, std::bind(&VelocityEstimationHandler::attitudeCallback, this, _1));
    //
    vn_imu_sub_ = create_subscription<vectornav_msgs::msg::ImuGroup>("vn_200/raw/imu",
        sensor_qos, std::bind(&VelocityEstimationHandler::imuCallback, this, _1));
    //
    front_wheel_encoder_sub_ = create_subscription<custom_msgs::msg::WheelSpeed>("canbus/front_hall_sensors",
        sensor_qos, std::bind(&VelocityEstimationHandler::frontWheelSpeedCallback, this, _1));
    //
    rear_wheel_encoder_sub_ = create_subscription<custom_msgs::msg::WheelSpeed>("canbus/rear_hall_sensors",
        sensor_qos, std::bind(&VelocityEstimationHandler::rearWheelSpeedCallback, this, _1));
    //
    steering_sub_ = create_subscription<custom_msgs::msg::SteeringAngle>("canbus/steering_angle",
        sensor_qos, std::bind(&VelocityEstimationHandler::steeringCallback, this, _1));
    //
    master_sub_ = create_subscription<custom_msgs::msg::NodeSync>("saltas_clock", 10,
        std::bind(&VelocityEstimationHandler::masterCallback, this, _1));
}

void VelocityEstimationHandler::publishResults() {
    constexpr static std::array<int, 3> outputs{ StateVx, StateVy, StateVyaw };
    const StateVector pub_state{ estimator_.getState() };
    const StateMatrix pub_cov{ estimator_.getStateCovariance() };
    custom_msgs::msg::VelEstimation msg;

    msg.counter = global_index_;
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

int VelocityEstimationHandler::getNodeFrequency() {
    using namespace std::chrono_literals;

    // Instead of a timer we get the node frequency from the mater node with the following client request
    auto request{ std::make_shared<custom_msgs::srv::GetFrequencies::Request>() };
    int call_counter{ 0 };
    while (!cli_->wait_for_service(1s)) // and call_counter < 15)
    {
        if (!rclcpp::ok())
        {
            return 0;
        }
        RCLCPP_INFO(get_logger(), "Could not get node frequency. Master service not available, waiting...");
       // call_counter++;
    }
    // if (call_counter == 15)
    // {
    //     RCLCPP_ERROR(get_logger(), "Client call timeout, the service is not available. Check master node.");
    //     return 0;
    // }
    // Send empty request
    auto result{ cli_->async_send_request(request) };
    // Await for response (TODO: Set a timeout for response time)
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // If get successful response return the node frequency
        RCLCPP_INFO_STREAM(get_logger(), "Node frequency has been set to" << node_frequency_);
        return result.get()->velocity_estimation_frequency;
    }
    else
    {
        // Otherwise raise an error (TODO: should actually do something else, or handle the error)
        RCLCPP_ERROR(get_logger(), "Failed to get node frequency");
        return 0;
    }
}

Eigen::Matrix<double, 3, 3> VelocityEstimationHandler::getRotationMatrix(double roll, double pitch, double yaw) {
    // Returns the rotation matrix of the angles specified. Input angles should be in degrees.
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    Eigen::Matrix<double, 3, 3> rot_matrix{};
    rot_matrix <<
        std::cos(pitch) * std::cos(yaw), std::cos(pitch)* std::sin(yaw), -std::sin(pitch),
        -std::cos(roll) * std::sin(yaw) + std::sin(roll) * std::sin(pitch) * std::cos(yaw), std::cos(roll)* std::cos(yaw) + std::sin(roll) * std::sin(pitch) * std::sin(yaw), std::sin(roll)* std::cos(pitch),
        std::sin(roll)* std::sin(yaw) + std::cos(roll) * std::sin(pitch) * std::cos(yaw), -std::sin(roll) * std::cos(yaw) + std::cos(roll) * std::sin(pitch) * std::sin(yaw), std::cos(roll)* std::cos(pitch);
    return rot_matrix.transpose();
}

// ROS Callback Functions

void VelocityEstimationHandler::masterCallback(const custom_msgs::msg::NodeSync::SharedPtr msg) {
    if (!msg->exec_velocity)
    {
        return;
    }
    rclcpp::Time starting_time{ this->now() };
    global_index_ = msg->global_index;
    estimator_.setUpdateVector(updated_sensors_);
    // Check if no new measurements came this time step and use dead reckoning if true
    bool dead_reck{ false };
    if (std::none_of(updated_sensors_.begin(), updated_sensors_.end(), [](bool v) { return v; }))
    {
        dead_reck = true;
        // RCLCPP_WARN_STREAM(get_logger(), "No measurements received this time step. Dead-reckoning...");
    }
    estimator_.setMeasurements(measurement_vector_);
    estimator_.setInputVector(input_vector_);
    updated_sensors_.fill(false);

    // Run predict step.
    estimator_.predict();
    // Run the update step where the threshold is applied and subsets are created.
    if (!dead_reck) { estimator_.update(); }
    // Publish the new estimated state.
    publishResults();

    rclcpp::Duration total_time{ this->now() - starting_time };
    // RCLCPP_INFO_STREAM(get_logger(), "\n-- Execution Completed --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void VelocityEstimationHandler::velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg) {
    // When in mode 0 or mode 3 we still get messages in the topics but the values are 0, messing up the filter, so we ignore them instead
    if ((msg->insstatus.mode == 0) or (msg->insstatus.mode == 3))
    {
        return;
    }
    // Write measurements to corresponding node vector
    Eigen::Matrix<double, 3, 1> velocity_vec{};
    velocity_vec << static_cast<double>(msg->velbody.x), static_cast<double>(msg->velbody.y), static_cast<double>(msg->velbody.z);
    velocity_vec = vn_300_rotation_matrix_ * velocity_vec;
    if (std::isnan(velocity_vec(0)) or std::isnan(velocity_vec(1)))
    {
        return;
    }
    else
    {
        measurement_vector_(ObservationVx) = velocity_vec(0);
        measurement_vector_(ObservationVy) = velocity_vec(1);
        // Set the update vector indices
        updated_sensors_[VelocitySensor] = true;
    }
}

// void VelocityEstimationHandler::attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg) {
//     // NOTE: see if yaw, pitch, roll measurement could be useful for something
       // automatic calculation of euler angles for rotation matrices? Possibly it would only work for vn_300. 
// }

void VelocityEstimationHandler::imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg) {
    // Write measurements to corresponding node vector
    Eigen::Matrix<double, 3, 1> yaw_rate_vec{};
    yaw_rate_vec << static_cast<double>(msg->angularrate.x), static_cast<double>(msg->angularrate.y), static_cast<double>(msg->angularrate.z);
    yaw_rate_vec = vn_200_rotation_matrix_ * yaw_rate_vec;

    Eigen::Matrix<double, 3, 1> acceleration_vec{};
    acceleration_vec << static_cast<double>(msg->accel.x), static_cast<double>(msg->accel.y), static_cast<double>(msg->accel.z);
    acceleration_vec = vn_200_rotation_matrix_ * acceleration_vec;

    if (std::isnan(acceleration_vec(0)) or std::isnan(acceleration_vec(1)) or std::isnan(yaw_rate_vec(2)))
    {
        return;
    }
    else
    {
        measurement_vector_(ObservationVyaw) = yaw_rate_vec(2);
        measurement_vector_(ObservationAx) = acceleration_vec(0);
        measurement_vector_(ObservationAy) = acceleration_vec(1);
        // Set the update vector indices
        updated_sensors_[Accelerometer] = true;
        updated_sensors_[Gyroscope] = true;
    }
}

void VelocityEstimationHandler::frontWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg) {
    // We take the average of the two wheels
    measurement_vector_(ObservationVhall_front) = (static_cast<double>(msg->right_wheel) + static_cast<double>(msg->left_wheel)) / 2.0;
    updated_sensors_[FrontWheelEncoders] = true;
}

void VelocityEstimationHandler::rearWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg) {
    // We take the average of the two wheels
    measurement_vector_(ObservationVhall_rear) = (static_cast<double>(msg->right_wheel) + static_cast<double>(msg->left_wheel)) / 2.0;
    updated_sensors_[RearWheelEncoders] = true;
}

void VelocityEstimationHandler::steeringCallback(const custom_msgs::msg::SteeringAngle::SharedPtr msg) {
    // 3.17 is the gear ratio of the steering rack
    RCLCPP_INFO_STREAM(get_logger(), "Steering angle: " << static_cast<int>(msg->steering_angle));
    input_vector_(InputSteering) = static_cast<double>(static_cast<int>(msg->steering_angle)) * M_PI / 180.0 / 3.17;       // converted to rad
}

// Loads the node parameters from the .yaml file
void VelocityEstimationHandler::loadParameters() {
    declare_parameter<int>("frequency", 50);

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
        { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 });

    declare_parameter<double>("vectornav-vn-200.x", 0.0);
    declare_parameter<double>("vectornav-vn-200.y", 0.0);
    declare_parameter<double>("vectornav-vn-200.roll", 0.0);
    declare_parameter<double>("vectornav-vn-200.pitch", 0.0);
    declare_parameter<double>("vectornav-vn-200.yaw", 0.0);

    declare_parameter<double>("vectornav-vn-300.x", 0.0);
    declare_parameter<double>("vectornav-vn-300.y", 0.0);
    declare_parameter<double>("vectornav-vn-300.roll", 0.0);
    declare_parameter<double>("vectornav-vn-300.pitch", 0.0);
    declare_parameter<double>("vectornav-vn-300.yaw", 0.0);

    declare_parameter<double>("wheel_radius", 0.2);
    declare_parameter<double>("front_axle", 1.0);
    declare_parameter<double>("rear_axle", 1.0);
}
} // namespace ns_vel_est
