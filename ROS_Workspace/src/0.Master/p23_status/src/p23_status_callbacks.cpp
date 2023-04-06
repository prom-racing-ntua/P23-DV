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

    // void P23StatusNode::updateSLAMInformation(const custom_msgs::msg::Slam::SharedPtr msg)
    // {

    // }

    // void P23StatusNode::updateControlsInput(const custom_msgs::msg::Controls::SharedPtr msg)
    // {

        // Send updated inputs to VCU
        // PCtoVCU_controls();
    // }

    void P23StatusNode::checkSensors()
    {
        //For IMU, request from Vectornav the IMU Mode Status
        requestINSStatus();
    }

    void P23StatusNode::PCtoVCU_slow()
    {
        custom_msgs::msg::CanSystemState systemStateMsg;

         systemStateMsg.mission_finished = missionFinished;
         systemStateMsg.standstill = standstill;
         systemStateMsg.pc_error = pcError;
         systemStateMsg.lap_counter = currentLap;
         systemStateMsg.cones_count_actual = conesActual;
         systemStateMsg.cones_count_all = conesCountAll;

         canbus_system_state_publisher_->publish(systemStateMsg);
    }

    void P23StatusNode::PCtoVCU_medium()
    {
        /*
            Should send information that are semi-important to the car, mostly things that help with the traction control and such. These are:
            acc_lateral(m/s^2, scale 1/512), acc_long(m/s^2, scale 1/512) and yaw_rate(degrees/s, scale 1/128). Refresh rate is 20Hz

            Each one is 2 bytes (LSB then MSB)
        */
       custom_msgs::msg::CanVehicleVariables vehicleVariablesMsg;

       // These are obviously wrong, just a placeholder for the actual values.
       vehicleVariablesMsg.lat_accel = accelerationY/512;
       vehicleVariablesMsg.long_accel = accelerationX/512;
       vehicleVariablesMsg.yaw_rate = yawRate/128;
    
       canbus_vehicle_variables_publisher_->publish(vehicleVariablesMsg);
    }

    /*
        The actual PCtoVCU_fast is the one that send the information that are returned by the control node. These should be sent as soon as they are 
        ready and should not wait. The other ones are done on a timer basis (10Hz for Slow and 20Hz for Medium) as to not overflow the canbus channel.
    */

    void P23StatusNode::PCtoVCU_controls()
    {
        custom_msgs::msg::CanControlCommand controlCommandMsg;

        // Need to fill this out (when the control node is done)

        canbus_controls_publisher_->publish(controlCommandMsg);
    }
}