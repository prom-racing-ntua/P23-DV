import threading
import time
from custom_msgs.msg import HallSensors
from struct import unpack


class CANBusHandler(threading.Thread):
    def __init__(self, class_caller):
        threading.Thread.__init__(self, daemon = True)
        self.__parent_class = class_caller

    def run(self):
        while True:
            while True:
                msg = self.__parent_class.bus.recv()
                if msg is None:
                    continue

                msg_id = msg.arbitration_id
                timestamp = time.time()
                sec = timestamp // 1
                nano = timestamp - sec
                
                if msg_id == 0x300:
                    # Rear Hall Sensors
                    new_msg = HallSensors()
                    right_msb = unpack("i>", msg.data[0])
                    right_lsb = unpack("i>", msg.data[1])
                    rear_right = (right_msb<<8) + right_lsb
                    left_msb = unpack("i>", msg.data[2])
                    left_lsb = unpack("i>", msg.data[3])
                    rear_left = (left_msb<<8) + left_lsb
                    
                    new_msg.header.stamp.sec, new_msg.header.stamp.nanosec = sec, nano
                    new_msg.wheels = "rear"
                    new_msg.right, new_msg.left = rear_right, rear_left

                elif msg_id == 0x310:
                    # Front Hall Sensors
                    new_msg = HallSensors()
                    right_msb = unpack("i>", msg.data[6])
                    right_lsb = unpack("i>", msg.data[7])
                    front_right = (right_msb<<8) + right_lsb
                    left_msb = unpack("i>", msg.data[4])
                    left_lsb = unpack("i>", msg.data[5])
                    front_left = (left_msb<<8) + left_lsb

                    new_msg.header.stamp.sec, new_msg.header.stamp.nanosec = sec, nano
                    new_msg.wheels = "front"
                    new_msg.right, new_msg.left = front_right, front_left

                elif msg_id == 0x301:
                    # Break Pressure
                    new_msg = HallSensors()
                    
                    brake_threshold = 0 #enter the actual brake threshold
                    front_pressure = unpack("i>", msg.data[-1]) #enter  the byte for front brake pressure
                    rear_pressure = unpack("i>", msg.data[-1]) #enter  the byte for rear brake pressure
                    if front_pressure > brake_threshold or rear_pressure > brake_threshold:
                        brakes = 1
                    else:
                        brakes =0

                    new_msg.header.stamp.sec, new_msg.header.stamp.nanosec = sec, nano
                    new_msg.wheels = "brakes"
                    new_msg.right = new_msg.left = brakes

                else:
                    continue

                self.__parent_class.publisher.publish(new_msg)