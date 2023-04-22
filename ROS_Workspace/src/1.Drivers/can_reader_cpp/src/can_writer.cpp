#include "can_messages.h"
#include "can_reader.h"
#include <string.h>

#include "rclcpp/rclcpp.hpp"

namespace can_reader_namespace {

    int CanReader::writeBufferToSerial(uint8_t buffer[], size_t bytesToSend)
    {
        /*
            NOTE: This write method is 100% faulty and SHOULD NOT BE USED IN THE END. We need to ensure that
            all the bytes have been written. We are writing to a serial medium AND NOT A FILE. This is fixed
            by doing a while() loop and keeping an offset of the bytes send.
        */ 
  //      int writeCount = write(serial_port, &buffer, (bytesToSend)*sizeof(uint8_t));
    //    return writeCount;
    }

    void CanReader::sendSystemState(const custom_msgs::msg::CanSystemState::SharedPtr msg)
    {

        /*
            CANBUS FRAME: Buffer[0] = 0x00;
            Buffer[1] = DLC = 0x04;
            Buffer[2] = ...;
            Buffer[3] = cones_count_actual;
            Buffer[4] = cones_count_all(lsb);
            Buffer[5] = cones_count_all(msb);
        */
        uint8_t systemStateBuffer[6];

        // 0x00 Identifier   DLC = 0x04 -> 4 bytes to send/receive

        systemStateBuffer[0] = 0x00;
        systemStateBuffer[1] = 0x04;

        systemStateBuffer[2] = (msg->mission_finished << 7) + (msg->standstill << 6) + (msg->pc_error << 5);
        
        systemStateBuffer[3] = static_cast<uint8_t>(msg->cones_count_actual);
        systemStateBuffer[4] = static_cast<uint8_t>(msg->cones_count_all) & 0xff;
        systemStateBuffer[5] = (static_cast<uint8_t>(msg->cones_count_all) >> 8) & 0xff;
        writeBufferToSerial(systemStateBuffer, 6);
    }

    void CanReader::sendVehicleVariables(const custom_msgs::msg::CanVehicleVariables::SharedPtr msg)
    {   
        uint8_t vehicleVariablesBuffer[8];
        uint16_t lateralAcceleration, longitudinalAcceleration, yawRate;

        lateralAcceleration = msg->lat_accel;
        longitudinalAcceleration = msg->long_accel;
        yawRate = msg->yaw_rate;
        
        //ID and DLC
        vehicleVariablesBuffer[0] = 0x02;
        vehicleVariablesBuffer[1] = 0x06;

        vehicleVariablesBuffer[2] = lateralAcceleration & 0xff;
        vehicleVariablesBuffer[3] = (lateralAcceleration >> 8) & 0xff;
        vehicleVariablesBuffer[4] = longitudinalAcceleration & 0xff;
        vehicleVariablesBuffer[5] = (longitudinalAcceleration >> 8) & 0xff;
        vehicleVariablesBuffer[6] = yawRate & 0xff;
        vehicleVariablesBuffer[7] = (yawRate >> 8) & 0xff;

        writeBufferToSerial(vehicleVariablesBuffer, 8);        
    }

    void CanReader::sendControlsCommand(const custom_msgs::msg::CanControlCommand::SharedPtr msg)
    {
        uint8_t controlCommandBuffer[7];
        uint8_t speedTarget = msg->speed_target;
        uint8_t speedActual = msg->speed_actual;

        controlCommandBuffer[0] = 0x01;
        controlCommandBuffer[1] = 0x05;

        controlCommandBuffer[2] = speedTarget;
        controlCommandBuffer[3] = speedActual;

        writeBufferToSerial(controlCommandBuffer, 7);
        
    }
}
