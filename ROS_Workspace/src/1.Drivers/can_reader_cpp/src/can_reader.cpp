#include <iostream>
#include <string.h>
#include <chrono>

#include <fcntl.h>
#include <errno.h>
#include <termios.h> 
#include <unistd.h> 

#include "can_messages.h"
#include "can_reader.h"

#include "rclcpp/rclcpp.hpp"

using namespace std;

namespace can_reader_namespace {
    CanReader::CanReader() : Node("can_reader_cpp_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing CAN2USB node");
        
        baud_rate = declare_parameter("baud_rate", 9600);
        port_number = declare_parameter("port_number", 0);
        timeout = declare_parameter("timeout", 1);
        freq = declare_parameter("read_frequency", 150);
        period = 1000000/freq;

        auto sensor_qos {
            rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data)
        };

        front_wheel_encoder_pub_ = create_publisher<custom_msgs::msg::WheelSpeed>("canbus/front_hall_sensors", sensor_qos);
        rear_wheel_encoder_pub_ = create_publisher<custom_msgs::msg::WheelSpeed>("canbus/rear_hall_sensors", sensor_qos);
        steering_pub_ = create_publisher<custom_msgs::msg::SteeringAngle>("canbus/steering_angle", sensor_qos);
        braking_pub_ = create_publisher<custom_msgs::msg::BrakePressure>("canbus/brake_pressure", sensor_qos);

        setup_serial();
        timer = create_wall_timer(std::chrono::nanoseconds(period), std::bind(&CanReader::read_serial, this));
    }

    CanReader::~CanReader() {
        close(serial_port);
    }

    void CanReader::setup_serial() {
        struct termios tty;

        RCLCPP_INFO(get_logger(), "Initializing...");

        //For Development
        //serial_port = open("/tmp/vserial1", O_RDWR);
        //For Use on car
        char dev[13];
        sprintf(dev, "/dev/ttyACM%i", port_number);
        RCLCPP_INFO(get_logger(), "Opening: %s", dev);
        serial_port = open(dev, O_RDWR);
        RCLCPP_INFO(get_logger(), "Port %i Opened.",serial_port);

        if (tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_INFO(get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        }

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CS8; 
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        cfsetispeed(&tty, baud_rate);
        cfsetospeed(&tty, baud_rate);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_INFO(get_logger(),"Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }
        
        RCLCPP_INFO(get_logger(), "Writing 4 0bytes");
        unsigned char wbuf[4] = { 0x0, 0x0, 0x0, 0x0} ;
        int wr = write(serial_port, &wbuf, 4);

        RCLCPP_INFO(get_logger(), "Init OK!");
    }

    CanMessage* CanReader::extract_message(unsigned int *buf) {
        short id = buf[0] << 8 | buf[1];
        RCLCPP_INFO(get_logger(), "Got id: 0x%04X", id);
        switch (id)
        {
        case 0x310: {            
            return new FrontHallsMessage(buf+3, front_wheel_encoder_pub_);
            break;
        }
        case 0x4D1: {
            return new RearHallsMessage(buf+3, rear_wheel_encoder_pub_);
            break;
        }
        case 0x4D2: {
            return new SteeringMessage(buf+3, steering_pub_);
            break;
        }
        case 0x301: {
            return new BrakesMessage(buf+3, braking_pub_);
            break;
        }
        case 0x305: {
            return new TestHallMessage(buf+3);
            break;
        }
        case 0x03: {
            return new StatePubMessage(buf+3);
            break;
        }
        default:
            return nullptr;
            break;
        }

    }

    void CanReader::read2byte(unsigned char *from, unsigned int *to, int n) {
        for (int i=0; i<n; i++) {
            unsigned int x1;
            std::stringstream ss;
            ss << std::hex << from[i*2] << from[(i*2)+1];
            ss >> x1;
            to[i] = x1;
        }
    }

    void CanReader::read_serial() {
        

        //CHAT GPT FTW
        // I guess readfds is a set of file descriptors we want to monitor. so we add to the set
        // the fd/serial port of the can2usb. I dont understand the purpose of serial_port+1 on select()
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serial_port, &readfds); // fd is the file descriptor for the input stream you want to monitor 
        struct timeval tv;
        tv.tv_sec = 0; // seconds
        tv.tv_usec = timeout*1000; //microseconds

        int ready = select(serial_port+1, &readfds, NULL, NULL, &tv); // fd+1 is the maximum file descriptor number in the set plus one

        if (ready == -1) {
            RCLCPP_INFO(get_logger(), "ERROR!");
        } else if (ready == 0) {
            //RCLCPP_INFO(get_logger(), "TIMEOUT");
            // better log this: cout << "Timeout " << endl;
        } else {
            unsigned char *read_buf = new unsigned char[23];
            unsigned int *byte_array = new unsigned int[11];

            RCLCPP_INFO(get_logger(), "READING");
            int n = read(serial_port, read_buf, 23);
            
            read2byte(read_buf, byte_array, 11);
            CanMessage *can_message = extract_message(byte_array);
            if (can_message != NULL) {
                RCLCPP_INFO(get_logger(), "HANDLING MESSAGE");
                can_message->handler();
            } 
            delete can_message;
            delete []read_buf;
        }
    }
    
}