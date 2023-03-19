#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include "can_reader.h"
#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/wheel_speed.hpp"
#include "custom_msgs/msg/steering_angle.hpp"
#include "custom_msgs/msg/brake_pressure.hpp"

namespace can_reader_namespace {
    class CanMessage {
        public:
            short id;
            unsigned int *message;
        
            CanMessage(int id_, unsigned int *msg);
            virtual ~CanMessage() {};
            virtual void handler() = 0;
    };

    class FrontHallsMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub_;

            FrontHallsMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub);
            ~FrontHallsMessage() override;
            void handler() override;
    };

    class RearHallsMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub_;

            RearHallsMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub);
            ~RearHallsMessage();
            virtual void handler();
    };

    class SteeringMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr pub_;
            
            SteeringMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr pub);
            ~SteeringMessage();
            virtual void handler();
    };

    class BrakesMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr pub_;

            BrakesMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr pub);
            ~BrakesMessage();
            virtual void handler();
    };

    class TestHallMessage : public CanMessage {
        public:
            TestHallMessage(unsigned int *msg);
            ~TestHallMessage();
            virtual void handler();
    };

    class StatePubMessage : public CanMessage {
        public:
            StatePubMessage(unsigned int *msg);
            ~StatePubMessage();
            virtual void handler();
    };
}

#endif