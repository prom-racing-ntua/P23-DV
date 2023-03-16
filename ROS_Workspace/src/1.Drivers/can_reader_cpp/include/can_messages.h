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
            unsigned char *message;
        
            CanMessage(int id_, unsigned char *msg);
            virtual ~CanMessage() {};
            virtual void handler() = 0;
    };

    class FrontHallsMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub_;

            FrontHallsMessage(unsigned char *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub);
            ~FrontHallsMessage() override;
            void handler() override;
    };

    class RearHallsMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub_;

            RearHallsMessage(unsigned char *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub);
            ~RearHallsMessage();
            virtual void handler();
    };

    class SteeringMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr pub_;
            
            SteeringMessage(unsigned char *msg, rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr pub);
            ~SteeringMessage();
            virtual void handler();
    };

    class BrakesMessage : public CanMessage {
        public:
            rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr pub_;

            BrakesMessage(unsigned char *msg, rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr pub);
            ~BrakesMessage();
            virtual void handler();
    };

    class TestHallMessage : public CanMessage {
        public:
            TestHallMessage(unsigned char *msg);
            ~TestHallMessage();
            virtual void handler();
    };

    class StatePubMessage : public CanMessage {
        public:
            StatePubMessage(unsigned char *msg);
            ~StatePubMessage();
            virtual void handler();
    };
}

#endif