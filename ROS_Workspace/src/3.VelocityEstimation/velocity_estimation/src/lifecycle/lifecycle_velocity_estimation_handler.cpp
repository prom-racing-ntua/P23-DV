#include "lifecycle/lifecycle_velocity_estimation_handler.hpp"

#include "rmw/qos_profiles.h"
#include <rclcpp/qos.hpp>


namespace ns_vel_est
{
    LifecycleVelocityEstimationHandler::LifecycleVelocityEstimationHandler(): LifecycleNode("velocity_estimation"), estimator_{ this } {
        loadParameters();
        RCLCPP_WARN(get_logger(), "Launched Lifecycle Velocity Estimation node");
        RCLCPP_WARN(get_logger(), "Lifecycle Velocity Estimator is Online (but not configured)");
    }

    // Set node subscribers
    void LifecycleVelocityEstimationHandler::setSubscribers() {
        using std::placeholders::_1;

        // Setting the quality of service for the subscribers to sensor data. This has a smaller buffer
        // for the subscriber and best effort reliability. Go to rmw_qos_profile_sensor_data definition
        // and to ros2 documentation for full details. The KeepLast(5) argument is not used since it gets
        // overwritten afterwards, it is necessary for the QoSInitialization argument.
        auto sensor_qos{ rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data) };

        vn_velocity_sub_ = create_subscription<vectornav_msgs::msg::InsGroup>("vn_300/raw/ins",
            sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::velocityCallback, this, _1));
        //
        // vn_attitude_sub_ = create_subscription<vectornav_msgs::msg::AttitudeGroup>("vn_300/raw/attitude",
        // sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::attitudeCallback, this, _1));
        //
        vn_imu_sub_ = create_subscription<vectornav_msgs::msg::ImuGroup>("vn_200/raw/imu",
            sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::imuCallback, this, _1));
        //
        front_wheel_encoder_sub_ = create_subscription<custom_msgs::msg::WheelSpeed>("canbus/front_hall_sensors",
            sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::frontWheelSpeedCallback, this, _1));
        //
        rear_wheel_encoder_sub_ = create_subscription<custom_msgs::msg::WheelSpeed>("canbus/rear_hall_sensors",
            sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::rearWheelSpeedCallback, this, _1));
        //
        steering_sub_ = create_subscription<custom_msgs::msg::SteeringAngle>("canbus/steering_angle",
            sensor_qos, std::bind(&LifecycleVelocityEstimationHandler::steeringCallback, this, _1));
        //
        master_sub_ = create_subscription<custom_msgs::msg::NodeSync>("saltas_clock", 10,
            std::bind(&LifecycleVelocityEstimationHandler::masterCallback, this, _1));
    }

    void LifecycleVelocityEstimationHandler::publishResults() {
        constexpr static std::array<int, 3> outputs{ StateVx, StateVy, StateVyaw };
        const StateVector pub_state{ estimator_.getState() };
        const StateMatrix pub_cov{ estimator_.getStateCovariance() };
        custom_msgs::msg::VelEstimation msg;

        msg.global_index = global_index_;

        if (std::fabs(pub_state(StateVx)) < 0.05) \
            msg.velocity_x = 0.0;
        else \
            msg.velocity_x = pub_state(StateVx);

        if (std::fabs(pub_state(StateVy)) < 0.05) \
            msg.velocity_y = 0.0;
        else \
            msg.velocity_y = pub_state(StateVy);

        msg.yaw_rate = pub_state(StateVyaw);
        msg.acceleration_x = pub_state(StateAx);
        msg.acceleration_y = pub_state(StateAy);

        for (size_t i{ 0 }; i < outputs.size(); ++i)
        {
            for (size_t j{ 0 }; j < outputs.size(); ++j)
            {
                msg.variance_matrix[i * outputs.size() + j] = pub_cov(outputs[i], outputs[j]);
            }
        }

        pub_->publish(msg);
    }

    void LifecycleVelocityEstimationHandler::getNodeFrequency() {
        using namespace std::chrono_literals;

        // Instead of a timer we get the node frequency from the mater node with the following client request
        auto request = std::make_shared<custom_msgs::srv::GetFrequencies::Request>();

        RCLCPP_INFO(get_logger(), "Before wait for service");

        while (!cli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                // return;
            }
        }

        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedFuture;
        auto responseReceivedCallback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            node_frequency_ = result.get()->velocity_estimation_frequency;
            /* Unblock Semaphore */
            RCLCPP_INFO(get_logger(), "Before sem_post");
            sem_post(&clientSemaphore);
        };

        auto future_result = cli_->async_send_request(request, responseReceivedCallback);
    }

    Eigen::Matrix<double, 3, 3> LifecycleVelocityEstimationHandler::getRotationMatrix(double roll, double pitch, double yaw) {
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

    void LifecycleVelocityEstimationHandler::masterCallback(const custom_msgs::msg::NodeSync::SharedPtr msg) {
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

    void LifecycleVelocityEstimationHandler::velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg) {
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

    // void LifecycleVelocityEstimationHandler::attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg) {
    //     // NOTE: see if yaw, pitch, roll measurement could be useful for something
        // automatic calculation of euler angles for rotation matrices? Possibly it would only work for vn_300. 
    // }

    void LifecycleVelocityEstimationHandler::imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg) {
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

    void LifecycleVelocityEstimationHandler::frontWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg) {
        // We take the average of the two wheels
        double right_rpm{ static_cast<double>(msg->right_wheel) };
        double left_rpm{ static_cast<double>(msg->left_wheel) };

        double temp{ (right_rpm + left_rpm) / 2.0 };
        if ((left_rpm == 0.0) and (right_rpm != 0.0)) \
            measurement_vector_(ObservationVhall_front) = right_rpm;
        else if ((left_rpm != 0.0) and (right_rpm == 0.0)) \
            measurement_vector_(ObservationVhall_front) = left_rpm;
        else if (std::fabs(temp - measurement_vector_(ObservationVhall_front)) >= 80.0) \
            measurement_vector_(ObservationVhall_front) = 0.997 * measurement_vector_(ObservationVhall_front) + 0.003 * temp;
        else \
            measurement_vector_(ObservationVhall_front) = temp;

        updated_sensors_[FrontWheelEncoders] = true;
    }

    void LifecycleVelocityEstimationHandler::rearWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg) {
        // We take the average of the two wheels
        double right_rpm{ static_cast<double>(msg->right_wheel) };
        double left_rpm{ static_cast<double>(msg->left_wheel) };

        double temp{ (right_rpm + left_rpm) / 2.0 };
        if ((left_rpm == 0.0) and (right_rpm != 0.0)) \
            measurement_vector_(ObservationVhall_rear) = right_rpm;
        else if ((left_rpm != 0.0) and (right_rpm == 0.0)) \
            measurement_vector_(ObservationVhall_rear) = left_rpm;
        else if (std::fabs(temp - measurement_vector_(ObservationVhall_rear)) >= 80.0) \
            measurement_vector_(ObservationVhall_rear) = 0.997 * measurement_vector_(ObservationVhall_rear) + 0.003 * temp;
        else \
            measurement_vector_(ObservationVhall_rear) = temp;

        updated_sensors_[RearWheelEncoders] = true;
    }

    void LifecycleVelocityEstimationHandler::steeringCallback(const custom_msgs::msg::SteeringAngle::SharedPtr msg) {
        // 3.17 is the gear ratio of the steering rack
        // RCLCPP_INFO_STREAM(get_logger(), "Steering angle: " << static_cast<int>(msg->steering_angle));
        input_vector_(InputSteering) = static_cast<double>(static_cast<int>(msg->steering_angle)) * M_PI / 180.0 / 3.17;       // converted to rad
    }

    // Loads the node parameters from the .yaml file
    void LifecycleVelocityEstimationHandler::loadParameters() {
        declare_parameter<int>("frequency", 0);

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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // auto lifecycleVelocityEstimationNode = std::make_shared<ns_vel_est::LifecycleVelocityEstimationHandler>();
    ns_vel_est::LifecycleVelocityEstimationHandler lifecycleVelocityEstimationNode{};
    rclcpp::spin(lifecycleVelocityEstimationNode.get_node_base_interface());
    rclcpp::shutdown();

    // auto options{ rclcpp::ExecutorOptions() };
    // rclcpp::executors::MultiThreadedExecutor executor{ options, 4 };

    // executor.add_node(lifecycleVelocityEstimationNode);
    // executor.spin();

    rclcpp::shutdown();
    return 0;
}