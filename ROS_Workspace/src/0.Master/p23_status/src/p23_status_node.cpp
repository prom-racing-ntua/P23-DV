#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "p23_status_node.hpp"

/*
    TODO List:
    1. Create Publishers to send towards CANBUS
    2. Write a callback to receive Velocity Estimation Message and update Current Accel/Vel values
    3. Write a callback to receive SLAM message and update current Laps e.t.c.
    4. Convert data to be sent to VCU to byte arrays.
    5. Write the launch sripts and find a smart way to control them.
*/


namespace p23_status_namespace
{
    P23StatusNode::P23StatusNode() : Node("p23_status_node")
    {
        initializeNode();
        setSubscribers();
        setServices();
        setPublishers();
    }

    void P23StatusNode::initializeNode()
    {
        conesCountAll = 0;
        conesActual = 0;
        insMode = 2;
        currentLap = 0;
        missionFinished = false;
        currentASStatus = AS_OFF;
        currentMission = INSPECTION;
        currentDVStatus = STARTUP;

        /*
            Initialize speed/acceleration variables and such.
        */

        /*
            Initialize Timers. Slow VCU communication is set at 10Hz, Medium VCU communication is set at 20Hz. Controls communication
            should be done as soon as new control decisions are made (ASAP). Sensor Checkups are set at 2Hz (temporarily, we will see)
        */

        sensorCheckupTimer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&P23StatusNode::checkSensors, this));
        pcToVCU_slow_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&P23StatusNode::PCtoVCU_slow, this));
        pcToVCU_medium_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&P23StatusNode::PCtoVCU_medium, this));
    }

    void P23StatusNode::setSubscribers()
    {
        using std::placeholders::_1;
        canbus_mission_subscription_ = create_subscription<custom_msgs::msg::MissionSelection>(
            "canbus/mission_selection", 10, std::bind(&P23StatusNode::updateMission, this, _1));

        canbus_status_subscription_ = create_subscription<custom_msgs::msg::AutonomousStatus>(
            "canbus/as_status", 10, std::bind(&P23StatusNode::updateASStatus, this, _1));

        velocity_estimation_subscription_ = create_subscription<custom_msgs::msg::VelEstimation>(
            "velocity_estimation", 10, std::bind(&P23StatusNode::updateVelocityInformation, this, _1));
        
        // slam_subscription_ = create_subscription<custom_msgs::msg::Slam>(
        //     "", 10, std::bind(&P23StatusNode::updateSLAMInformation, this, _1));
    }

    void P23StatusNode::setPublishers()
    {
        // Set publisher towards CANBUS writer topic
        // canbus_slow_publisher_ = 
        // canbus_medium_publisher_ =
        // canbus_controls_publisher_ = 
    }

    void P23StatusNode::setServices()
    {
        // Set a service to change DV Status and to receive IMU Mode
        p23_status_client_ = create_client<custom_msgs::srv::DriverlessStatus>("lifecycle_manager/driverless_status");
        ins_status_client_ = create_client<custom_msgs::srv::CustomInsStatus>("velocity_estimation/ins_status_service");
    }

    void P23StatusNode::updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg)
    {
        currentMission = static_cast<Mission>(msg->mission_selected);

        switch(currentMission){
            case(ACCELERATION):
                // I think that the best way to do this is by forking and then running execve().
                break;
            case(SKIDPAD):
                break;
            case(TRACKDRIVE):
                break;
            case(EBS_TEST):
                break;
            case(INSPECTION):
                break;
            case(AUTOX):
                break;
            case(MANUAL):
                //bash script that shuts down pc in 1 minute (Tha doumee)
                break;
        }
        //Start the correct launch file correspodning to the mission
    }

    void P23StatusNode::updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg)
    {
        currentASStatus = static_cast<AS_Status>(msg->as_status);
    }

    void P23StatusNode::updateVelocityInformation(const custom_msgs::msg::VelEstimation::SharedPtr msg)
    {

    }

    void P23StatusNode::checkSensors()
    {
        //For IMU, request from Vectornav the IMU Mode Status
        requestINSStatus();
    }

    void P23StatusNode::PCtoVCU_slow()
    {
        /*
            Should send information on a 10Hz refresh rate. The info sent are the ones that are not heavily important for the car to function.
            These are: MissionFinished(bool), LapCounter(unsigned 4-bit), Standstill(bool), PC Error(bool), Cones Count actual(unsigned) and Cones Count All(unsigned).
            First 4 are 1 byte, Cones count actual is 1 byte and cones count all is 2 bytes
        */

       // TODO: Implement this to the CANBUS writer
    }

    void P23StatusNode::PCtoVCU_medium()
    {
        /*
            Should send information that are semi-important to the car, mostly things that help with the traction control and such. These are:
            acc_lateral(m/s^2, scale 1/512), acc_long(m/s^2, scale 1/512) and yaw_rate(degrees/s, scale 1/128). Refresh rate is 20Hz

            Each one is 2 bytes (LSB then MSB)
        */

       // TODO: Implement this to the CANBUS writer
    }

    /*
        The actual PCtoVCU_fast is the one that send the information that are returned by the control node. These should be sent as soon as they are 
        ready and should not wait. The other ones are done on a timer basis (10Hz for Slow and 20Hz for Medium) as to not overflow the canbus channel.
    */

    void P23StatusNode::PCtoVCU_controls()
    {

    }

    void P23StatusNode::requestINSStatus()
    {
        using namespace std::chrono_literals;

        auto request = std::make_shared<custom_msgs::srv::CustomInsStatus::Request>();
        
        while (!ins_status_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            RCLCPP_INFO(get_logger(), "Velocity Estimation INS Service not available, waiting...");
        }

        // Send INS Mode request
        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::CustomInsStatus>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            insMode = result.get()->ins_mode;
            RCLCPP_INFO(get_logger(), "Received INS Mode from Vel Est");
        };

        auto future_result = ins_status_client_->async_send_request(request, response_received_callback);
    }

    void P23StatusNode::changeDVStatus(DV_Status newStatus)
    {
        using namespace std::chrono_literals;

        // Create a service to communicate with the Lifecycle Manager to change the status to whatever is specified
        auto request = std::make_shared<custom_msgs::srv::DriverlessStatus::Request>();

        request->new_status.id = newStatus;
        request->new_status.label = driverlessStatusList[newStatus];

        if ((newStatus == DV_READY) || (newStatus == DV_DRIVING)) {
            RCLCPP_INFO(get_logger(), "Want to change to DV_READY or DV_DRIVING --> CHECK INS MODE");
            while(insMode != 2) {
                RCLCPP_INFO(get_logger(), "IMU Still not in mode 2. Current mode: %u",insMode);
            }   
        }

        while (!p23_status_client_->wait_for_service(1s))
        {
            //pinw polla kokoria
            if (!rclcpp::ok())
            {
                return;
            }
            RCLCPP_INFO(get_logger(), "Lifecycle Manager Service not available, waiting...");
        }

        // Send request to Lifecycle Manager
        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::DriverlessStatus>::SharedFuture;
        auto response_received_callback = [this, newStatus](ServiceResponseFuture future) {
            auto result = future.get();
            RCLCPP_INFO(get_logger(), "Received result from Lifecycle Manager Service");

            bool success = result.get()->success;

            if (success)
            {
                RCLCPP_INFO(get_logger(), "Driverless Status successfully changed to: %s", driverlessStatusList[newStatus].c_str());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Could not change Driverless Status to: %s", driverlessStatusList[newStatus].c_str());
            }
        };

        auto future_result = p23_status_client_->async_send_request(request, response_received_callback);
    }
}