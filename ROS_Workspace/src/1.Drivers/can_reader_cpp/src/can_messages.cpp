#include <iostream>

#include "can_messages.h"

using namespace std;

namespace can_reader_namespace {
    CanMessage::CanMessage(int id_, unsigned int *msg) {
        id = id_;
        message = msg;
    }

    FrontHallsMessage::FrontHallsMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub) 
        : CanMessage(0x0310, msg), pub_(pub) {}
    RearHallsMessage::RearHallsMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::WheelSpeed>::SharedPtr pub) 
        : CanMessage(0x4D1, msg), pub_(pub) {} 
    SteeringMessage::SteeringMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::SteeringAngle>::SharedPtr pub) 
        : CanMessage(0x4D2, msg), pub_(pub) {} 
    BrakesMessage::BrakesMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::BrakePressure>::SharedPtr pub) 
        : CanMessage(0x301, msg), pub_(pub) {} 
    TestHallMessage::TestHallMessage(unsigned int *msg) : CanMessage(0x305, msg) {} 
    StatePubMessage::StatePubMessage(unsigned int *msg) : CanMessage(0x03, msg) {}
    MissionMessage::MissionMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::Mission>::SharedPtr pub)
        : CanMessage(0x500, msg), pub_(pub) {}
    AutonomousStatusMessage::AutonomousStatusMessage(unsigned int *msg, rclcpp::Publisher<custom_msgs::msg::AutonomousStatus>::SharedPtr pub)
        : CanMessage(0x501, msg), pub_(pub) {}

    FrontHallsMessage::~FrontHallsMessage() {}
    RearHallsMessage::~RearHallsMessage() {}
    SteeringMessage::~SteeringMessage() {}
    BrakesMessage::~BrakesMessage() {}
    TestHallMessage::~TestHallMessage() {}
    StatePubMessage::~StatePubMessage() {}
    MissionMessage::~MissionMessage() {}
    AutonomousStatusMessage::~AutonomousStatusMessage() {}

    void FrontHallsMessage::handler() {
        custom_msgs::msg::WheelSpeed msg;
        msg.left_wheel = message[4] << 8 | message[5];
        msg.right_wheel = message[6] << 8 | message[7];

        msg.actual_speed = (msg.left_wheel + msg.right_wheel) / 2 / 9.5493 * 0.1995;

        pub_->publish(msg);        
    }

    void RearHallsMessage::handler() {
        custom_msgs::msg::WheelSpeed msg;
        msg.left_wheel = message[4] << 8 | message[5];
        msg.right_wheel = message[6] << 8 | message[7];

        msg.actual_speed = (msg.left_wheel + msg.right_wheel) / 2 / 9.5493 * 0.1995;

        pub_->publish(msg);
    }

    void SteeringMessage::handler() {
        custom_msgs::msg::SteeringAngle msg;
        msg.steering_angle = message[4];

        pub_->publish(msg);
    }

    void BrakesMessage::handler() {
        custom_msgs::msg::BrakePressure msg;
        msg.front_cylinder = message[2];
        msg.rear_cylinder = message[3];

        pub_->publish(msg);
    }

    void TestHallMessage::handler() {
        std::cout << "TH Handler " << id << std::endl;
    }

    void StatePubMessage::handler() {
        std::cout << "SP Handler " << id << std::endl;
    }

    void MissionMessage::handler() {
        custom_msgs::msg::Mission msg;
        msg.id = message[2];

        pub_->publish(msg);
    }
    
    void AutonomousStatusMessage::handler() {
        custom_msgs::msg::AutonomousStatus msg;
        msg.id = message[2];

        pub_->publish(msg);
    }

}