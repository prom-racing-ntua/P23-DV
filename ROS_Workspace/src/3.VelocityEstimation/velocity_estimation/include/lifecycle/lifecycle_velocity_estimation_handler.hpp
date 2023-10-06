#ifndef LIFECYCLE_VELOCITY_ESTIMATION_HANDLER_HPP
#define LIFECYCLE_VELOCITY_ESTIMATION_HANDLER_HPP

#include <chrono>
#include <functional>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// Include custom vectornav messages
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/rx_steering_angle.hpp"
#include "custom_msgs/msg/rx_wheel_speed.hpp"
#include "custom_msgs/msg/node_sync.hpp"
#include "custom_msgs/srv/get_frequencies.hpp"

#include <semaphore.h>
#include <pthread.h>  /* My beloved :3 */

#include "velocity_estimation.h"

namespace ns_vel_est
{
    class Logger
    {
    private:
        FILE *file;
        int run_idx;
        std::string name;
    public:
        Logger();
        void init(std::string name);
        ~Logger();
        std::string check()const;
        void log(double timestamp, int type, int index);
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class LifecycleVelocityEstimationHandler: public rclcpp_lifecycle::LifecycleNode {
    private:
        int node_frequency_;
        unsigned long global_index_;
        VelocityEstimator<LifecycleVelocityEstimationHandler> estimator_;

        // ROS Subscriber members
        rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr vn_velocity_sub_;        // vn-300
        // rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr vn_attitude_sub_;   // vn-300
        rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn_imu_sub_;             // vn-200
        rclcpp::Subscription<custom_msgs::msg::RxWheelSpeed>::SharedPtr wheel_encoder_sub_;     // can-usb
        rclcpp::Subscription<custom_msgs::msg::RxSteeringAngle>::SharedPtr steering_sub_;       // can-usb

        // ROS Subscriber to the master node to sync velocity estimation and perception. The frequency of the node is determined by
        // this subscriber and is set in the master node config.
        rclcpp::Subscription<custom_msgs::msg::NodeSync>::SharedPtr master_sub_;

        // ROS Publisher to output results to the velocity estimation topic
        rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::VelEstimation>::SharedPtr pub_;

        // ROS Client to get the node frequency from the master node
        rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedPtr cli_;

        std::array<bool, SensorSize> updated_sensors_;
        ObservationVector measurement_vector_;
        InputVector input_vector_;

        Logger timestamp_log;
        double pub_time_1, pub_time_2;

        Eigen::Matrix<double, 3, 3> vn_200_rotation_matrix_;
        Eigen::Matrix<double, 3, 3> vn_300_rotation_matrix_;

        Eigen::Matrix<double, 3, 3> getRotationMatrix(double roll, double pitch, double yaw = 0.0);

        void loadParameters();
        void setSubscribers();
        
        // Callbacks
        void masterCallback(const custom_msgs::msg::NodeSync::SharedPtr msg);
        void velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg);
        // void attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg);
        void imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg);
        void wheelSpeedCallback(const custom_msgs::msg::RxWheelSpeed::SharedPtr msg);
        void steeringCallback(const custom_msgs::msg::RxSteeringAngle::SharedPtr msg);

    public:
        explicit LifecycleVelocityEstimationHandler();
        //~LifecycleVelocityEstimationHandler();
        void publishResults();
        void getNodeFrequency();

        // Returns the frequency at witch the node executes the EKF algorithm
        // int getNodeFrequency() const { return node_frequency_; }
        
    protected:
        ns_vel_est::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
        ns_vel_est::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
        ns_vel_est::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        ns_vel_est::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        ns_vel_est::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
        ns_vel_est::CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    };
} // namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_HANDLER_HPP