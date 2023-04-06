#ifndef CAN_READER_H
#define CAN_READER_H

#include "can_messages.h"
#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/wheel_speed.hpp"
#include "custom_msgs/msg/steering_angle.hpp"
#include "custom_msgs/msg/brake_pressure.hpp"
#include "custom_msgs/msg/mission.hpp"
#include "custom_msgs/msg/autonomous_status.hpp"

#include "custom_msgs/msg/can_vehicle_variables.hpp"
#include "custom_msgs/msg/can_control_command.hpp"
#include "custom_msgs/msg/can_system_state.hpp"

namespace can_reader_namespace {    
    class CanMessage;
    
    class CanReader: public rclcpp::Node{
        private:
            //Serial port config members
            int baud_rate;
            char port_number;
            int serial_port;
            int timeout;
            int freq;
            int period;
            fd_set readfds;
            struct timeval tv;
            rclcpp::TimerBase::SharedPtr timer;
            
            //ROS members
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr front_wheel_encoder_pub_;
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr rear_wheel_encoder_pub_;
            rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr steering_pub_;
            rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr braking_pub_;
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr test_hall_pub_;
            rclcpp::Publisher<custom_msgs::msg::AutonomousStatus>::SharedPtr autonomous_status_pub_;
            rclcpp::Publisher<custom_msgs::msg::Mission>::SharedPtr mission_pub_;

            rclcpp::Subscription<custom_msgs::msg::CanSystemState>::SharedPtr system_state_sub_;
            rclcpp::Subscription<custom_msgs::msg::CanVehicleVariables>::SharedPtr vehicle_variables_sub_;
            rclcpp::Subscription<custom_msgs::msg::CanControlCommand>::SharedPtr controls_command_sub_;

            // do i need this?
            //rclcpp::Subscription<custom_msgs::msg::NodeSync>::SharedPtr master_sub_;

            // ROS Client to get the node frequency from the master node
            // do i need this?
            //rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedPtr cli_;

            //Functions
            bool setup_serial();
            unsigned int charToInt(unsigned char r);
            
            /*
                Functions that send data to the VCU. There should not be anything else to send.
                Everything is handled from the P23 Status Node. CAN Node should just convert to bytes
                and write it to the serial port.
            */

            void sendSystemState(const custom_msgs::msg::CanSystemState::SharedPtr msg);
            void sendVehicleVariables(const custom_msgs::msg::CanVehicleVariables::SharedPtr msg);
            void sendControlsCommand(const custom_msgs::msg::CanControlCommand::SharedPtr msg);
            int writeBufferToSerial(uint8_t buffer[], size_t bytesToSend);

            CanMessage * extract_message(unsigned int *buf);
            void read2byte(unsigned char *buf, unsigned int *to, int n);
            
        public:
            CanReader();
            ~CanReader();
            
            void read_serial();
    };
    
} // namespace can_reader_namespace
#endif