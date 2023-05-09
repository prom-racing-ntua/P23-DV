#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "p23_status_node.hpp"


namespace p23_status_namespace
{
    void P23StatusNode::updateVelocityInformation(const custom_msgs::msg::VelEstimation::SharedPtr msg)
    {
        // Speed Actual
        velocityX = static_cast<double>(msg->velocity_x);
        velocityY = static_cast<double>(msg->velocity_y);
        yawRate = static_cast<double>(msg->yaw_rate);

        // Acceleration Values
        accelerationX = static_cast<double>(msg->acceleration_x);
        accelerationY = static_cast<double>(msg->acceleration_y);
    }

    void P23StatusNode::updateSLAMInformation(const custom_msgs::msg::PoseMsg::SharedPtr msg)
    {
        conesCountAll = msg->cones_count_all;
        currentLap = msg->lap_count;

        if (currentLap >= maxLaps)
        {
            // Go to Mission Finished state and start braking
        }
    }

    void P23StatusNode::checkSensors()
    {
        //For IMU, request from Vectornav the IMU Mode Status
        requestINSStatus();
    }

    void P23StatusNode::sendSystemState()
    {
        custom_msgs::msg::TxSystemState systemStateMsg;

        systemStateMsg.mission_finished = missionFinished;
        systemStateMsg.standstill = standstill;
        systemStateMsg.pc_error = pcError;
        systemStateMsg.lap_counter = currentLap;
        systemStateMsg.cones_count_actual = conesActual;
        systemStateMsg.cones_count_all = conesCountAll;

        canbus_system_state_publisher_->publish(systemStateMsg);
    }

    void P23StatusNode::sendVehicleVariables()
    {
        /*
            Should send information that are semi-important to the car, mostly things that help with
            the traction control and such.
        */
        custom_msgs::msg::TxVehicleVariables vehicleVariablesMsg;

        // vehicleVariablesMsg.lat_accel = accelerationY;
        // vehicleVariablesMsg.long_accel = accelerationX;
        // vehicleVariablesMsg.yaw_rate = yawRate;
        
        canbus_vehicle_variables_publisher_->publish(vehicleVariablesMsg);
    }

    void P23StatusNode::updateControlsInput(const custom_msgs::msg::MpcToCan::SharedPtr msg)
    {
        custom_msgs::msg::TxControlCommand controlCommandMsg;
        motorTorqueTarget = msg->mt;
        steeringAngleTarget = msg->sa;
        brakeHydraulicTarget = msg->bp;

        controlCommandMsg.speed_actual = velocityX;
        // controlCommandMsg.speed_target = ;

        controlCommandMsg.motor_torque_target = motorTorqueTarget;
        controlCommandMsg.steering_angle_target = steeringAngleTarget;
        controlCommandMsg.brake_pressure_target = brakeHydraulicTarget;
        
        canbus_controls_publisher_->publish(controlCommandMsg);
    }
}