import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_msgs.msg import String, UInt32
from pynput import keyboard
import time
from custom_msgs.msg import *


# STEERING_STEP = 1.0   #[mm]
# PRESS_STEP = 1.0    #[bar]
# TORQUE_STEP = 1.0       # [N*m]

# COMMAND_FREQUENCY = 10  # [Hz]
# MAX_TORQUE = 5.0    # [N*m]
# MIN_TORQUE = 0.0  #[N*m]
# MAX_STEERING = 24.0     # [mm]
# MIN_STEERING = -24.0
# MAX_PRESS = 20.0    #[bar]
# MIN_PRESS = 0.0    #[bar]

class BenchController(Node):
    def __init__(self,name=None) -> None:
        super().__init__('bench_controller')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.load_from_config()
        
        self._steering_command = self.declare_parameter('steering', 0.0).value  #[mm]
        self._brake_command = self.declare_parameter('brake', 0.0).value        #[bar]
        self._torque_command = self.declare_parameter('torque', 0.0).value      #[Nm]
        #set publishers
        self._command_publisher = self.create_publisher(TxControlCommand ,'/control_commands', qos_profile=10) #used for sure
        self._state_publisher = self.create_publisher(TxSystemState ,'/system_state', 10) 
        #set subscribers (to be used for datalogging and warning msgs)
        self.sub_code = self.create_subscription(UInt32, 'key_pressed', self.on_code,10)
        self._steering_sub = self.create_subscription(RxSteeringAngle, 'canbus/steering_angle', self.set_steering, 10)
        self._motor_sub = self.create_subscription(RxVehicleSensors, 'canbus/sensor_data', self.set_motor, 10)
        self.timer = self.create_timer(1/self.publish_frequency, self.timer_callback)   

    def timer_callback(self):
        msg = TxControlCommand()
        msg.brake_pressure_target = self._brake_command
        msg.steering_angle_target = self._steering_command
        msg.motor_torque_target = self._torque_command
        msg.speed_actual = 0
        msg.speed_target = 0
        self.get_logger().info(f'Brake pressure is {msg.brake_pressure_target} bar')
        self.get_logger().info(f'Steering Angle is {msg.steering_angle_target} rad')
        self.get_logger().info(f'Motor Torque {msg.motor_torque_target} Nm')
        self._command_publisher.publish(msg)
    
    def on_code(self, msg):
        if msg.data == keyboard.Key.f1.value.vk:
            self.logger.info("\n".join([
                'Use the arrow keys to change speed.',
                '[F1] = Show this help',
                '[Up]/[Down] = Increase and decrease motor torque',
                '[Left]/[Right] = Steering',
                '[ShiftL]/[ShiftR] = Decrease and increase brake pressure'
            ]))
        #torque_handling
        elif msg.data == keyboard.Key.up.value.vk:
            self.logger.info('Increasing motor torque')
            self._torque_command += self.torque_step
            if(self._torque_command>self.max_torque): 
                self._torque_command=self.max_torque
                self.logger.warn("Trying to exceed maximum torque set at {} Nm.".format(self.max_torque))
        elif msg.data == keyboard.Key.down.value.vk:
            self.logger.info('Decreasing motor torque')
            self._torque_command -= self.torque_step
            if(self._torque_command<self.min_torque): 
                self._torque_command=self.min_torque
                self.logger.warn("Trying to exceed minimum torque set at {} Nm.".format(self.min_torque))
        #steering handling
        elif msg.data == keyboard.Key.left.value.vk:
            self.logger.info('Steering left')
            self._steering_command -= self.steering_step
            if(self._steering_command<self.min_steering): 
                self._steering_command=self.min_steering
                self.logger.warn("Trying to exceed maximum steering rack displ. set at {} mm.".format(self.min_steering))
        elif msg.data == keyboard.Key.right.value.vk:
            self.logger.info('Steering right')
            self._steering_command += self.steering_step
            if(self._steering_command>self.max_steering): 
                self._steering_command=self.max_steering
                self.logger.warn("Trying to exceed minimum steering rack displ. set at {} mm.".format(self.max_steering))
        #pressure handling
        elif msg.data == keyboard.Key.shift_l.value.vk:
            self.logger.info('Decreasing Brake Pressure')
            self._brake_command -= self.press_step
            if(self._brake_command<self.min_press): 
                self._brake_command=self.min_press
                self.logger.warn("Trying to exceed minimum brake pressure set at {} bar.".format(self.min_press))
        elif msg.data == keyboard.Key.shift_r.value.vk:
            self.logger.info('Increasing Brake Pressure')
            self._brake_command += self.press_step
            if(self._brake_command>self.max_press): 
                self._brake_command=self.max_press
                self.logger.warn("Trying to exceed maximum brake pressure set at {} bar.".format(self.max_press))
        #other key pressed
        else:
            self.logger.debug('Key ignored: {}'.format(msg.data))
    
    def set_steering(self, msg:RxSteeringAngle) -> None:
        self._steering_angle = msg.steering_angle
        return

    def set_motor(self, msg:RxVehicleSensors) -> None:
        self._actual_torque = msg.motor_torque_actual
        return
    
    @property
    def logger(self):
        return self.node.get_logger()
    
    def spin(self):
        while rclpy.ok():
            rclpy.spin(self.node)
    
    def load_from_config(self) -> None:
        self.max_torque = self.declare_parameter('max_torque', 10.0).value
        self.min_torque = self.declare_parameter('min_torque', 0.0).value
        self.torque_step = self.declare_parameter('torque_step', 1.0).value
        self.max_steering = self.declare_parameter('max_steering', 24.0).value
        self.min_steering = self.declare_parameter('min_steering', -24.0).value
        self.steering_step = self.declare_parameter('steering_step', 1.0).value
        self.max_press = self.declare_parameter('max_press', 20.0).value
        self.min_press = self.declare_parameter('min_press', 0.0).value
        self.press_step = self.declare_parameter('press_step', 1.0).value
        self.publish_frequency = self.declare_parameter('publish_frequency',40.0).value        

def main(args=None):
    rclpy.init(args=args)
    # Spin Node
    bench_controller_node = BenchController()
    executor = SingleThreadedExecutor()
    rclpy.spin(bench_controller_node, executor)
    try:
        bench_controller_node.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        bench_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()