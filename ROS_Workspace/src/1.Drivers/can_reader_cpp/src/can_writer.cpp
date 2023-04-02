#include "can_messages.h"
#include "can_reader.h"
#include <string.h>

#include "rclcpp/rclcpp.hpp"

namespace can_reader_namespace {

    int CanReader::writeBufferToSerial(int id, int buffer[], size_t bytesToSend)
    {

        /*
            This is not tested and bound to change...
            Will test it with the CAN2USB module
        */
       
        int messageToSend[bytesToSend+1];

        // Set the ID of the Canbus message
        messageToSend[0] = id;

        // Copy the rest of the message
        memcpy(&messageToSend[1], buffer, bytesToSend*sizeof(int));

        //Write message to serial port

        /*
            NOTE: This write method is 100% faulty and SHOULD NOT BE USED IN THE END. We need to ensure that
            all the bytes have been written. We are writing to a serial medium AND NOT A FILE. This is fixed
            by doing a while() loop and keeping an offset of the bytes send.
        */ 
        int writeCount = write(serial_port, &messageToSend, (bytesToSend+1)*sizeof(int));

        return writeCount;

    }

    void CanReader::sendSystemState(const custom_msgs::msg::CanSystemState::SharedPtr msg)
    {
        

    }

    void CanReader::sendVehicleVariables(const custom_msgs::msg::CanVehicleVariables::SharedPtr msg)
    {
        
    }

    void CanReader::sendControlsCommand(const custom_msgs::msg::CanControlCommand::SharedPtr msg)
    {
        
    }
}