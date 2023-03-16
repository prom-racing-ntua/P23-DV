#ifndef CAN_READER_H
#define CAN_READER_H

#include "can_messages.h"
#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/wheel_speed.hpp"
#include "custom_msgs/msg/steering_angle.hpp"
#include "custom_msgs/msg/brake_pressure.hpp"

namespace can_reader_namespace {    
    class CanMessage;
    
    class CanReader: public rclcpp::Node{
        private:
            //Serial port config members
            int baud_rate;
            int port_number;
            int serial_port;
            int timeout;
            int freq;
            int period;
            rclcpp::TimerBase::SharedPtr timer;
            
            //ROS members
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr front_wheel_encoder_pub_;
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr rear_wheel_encoder_pub_;
            rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr steering_pub_;
            rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr braking_pub_;
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr test_hall_pub_;

            // do i need this?
            //rclcpp::Subscription<custom_msgs::msg::NodeSync>::SharedPtr master_sub_;

            // ROS Client to get the node frequency from the master node
            // do i need this?
            //rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedPtr cli_;




            //Functions
            void setup_serial();
            void write_serial(CanMessage &message);
            CanMessage * extract_message(unsigned char *buf);
            
        public:
            CanReader();
            ~CanReader();
            
            void read_serial();
    };
    
} // namespace can_reader_namespace
#endif