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

        //For Development
        //serial_port = open("/tmp/vserial1", O_RDWR);
        //For Use on car
        serial_port = open("/dev/ttyUSB"+port_number, O_RDWR);

        if (tcgetattr(serial_port, &tty) != 0) {
            printf("Error %i from tcgetattr: %s", errno, strerror(errno));
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
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }

        unsigned char wbuf[4] = { 0x0, 0x0, 0x0, 0x0} ;
        int wr = write(serial_port, &wbuf, 4);
    }

    CanMessage* CanReader::extract_message(unsigned char *buf) {
        short id = buf[0] << 8 | buf[1];
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

    void CanReader::read_serial() {
        unsigned char *read_buf = new unsigned char[11];

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
            //error log
        } else if (ready == 0) {
            // better log this: cout << "Timeout " << endl;
        } else {
            int n = read(serial_port, read_buf, 11);

            CanMessage *can_message = extract_message(read_buf);
            can_message->handler();
            delete can_message;
            delete []read_buf;
        }
    }
    
}