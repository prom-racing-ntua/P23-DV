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

    // void P23StatusNode::updateSLAMInformation()
    // {

    // }

    // void P23StatusNode::updateControlsInput()
    // {

        // Send updated inputs to VCU
        // PCtoVCU_controls();
    // }

    void P23StatusNode::checkSensors()
    {
        //For IMU, request from Vectornav the IMU Mode Status
        // requestINSStatus();
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
}