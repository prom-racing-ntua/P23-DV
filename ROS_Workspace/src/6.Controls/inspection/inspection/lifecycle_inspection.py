import rclpy
from rclpy.node import Node
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy._rclpy_pybind11 import InvalidHandle
from pynput import keyboard
from std_msgs.msg import String, UInt32
import time
from custom_msgs.msg import *
from custom_msgs.srv import SetTotalLaps
from math import sin, cos, pi, radians

COMMAND_FREQUENCY = 40          # [Hz]
TORQUE_COMMAND = 0.0            # [N*m]
MAX_STEERING = radians(15.0)    # [rad]
STEERING_PERIOD = 8             # [sec]
MISSION_DURATION = 24           # [sec]

class InspectionMission(Node):
    def __init__(self,name=None) -> None:
        super().__init__('inspection')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.declare_parameters(
            namespace='',
            parameters=[
            ('mode', 'bench'),
            ('max_torque', 10.0),
            ('min_torque', 0.0),
            ('torque_step', 1.0),
            ('max_steering', 24.0),
            ('min_steering', -24.0),
            ('steering_step', 1.0),
            ('max_press',20.0),
            ('min_press',0.0),
            ('press_step',1.0),
            ('publish_frequency',40.0),
            ('kp',-100.0),
            ('ki',0.0),
            ('kd',10.0),
            ('dt',0.01),
            ('minvel',-1000.0),
            ('maxvel',+1000.0),
            ('motor_by_function', False),
            ('motor_function', 'sine'),
            ('motor_frequency', 1.0),
            ('steer_by_function', False),
            ('steer_function', 'sine'),
            ('steer_frequency', 1.0),
            ('brake_by_function', False),
            ('brake_function', 'sine'),
            ('brake_frequency', 1.0),
            ]
        ) 
        #changes
        self.get_logger().warn("Inspection node created")

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self._command_publisher = self.create_publisher(TxControlCommand ,'/control_commands', 10)
        self._steering_publisher = self.create_publisher(TxSteeringParams,'/steering_params',10)
        self._state_publisher = self.create_publisher(TxSystemState ,'/system_state', 10)

        self._steering_sub = self.create_subscription(RxSteeringAngle, 'canbus/steering_angle', self.set_steering, 10)
        self._motor_sub = self.create_subscription(RxVehicleSensors, 'canbus/sensor_data', self.set_motor, 10)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if(self.mode=="inspection"):
            self.load_from_config()
            self.mission_finished = False
            self._steering_angle = 0.0
            self._actual_torque = 0.0
            # self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
            self.get_logger().warn("Inspection Configured on mode {}".format(self.mode))
        else:
            self.load_from_config()
            self._steering_command = 0.0  #[mm]
            self._brake_command = 0.0  #[bar]
            self._torque_command = 0.0  #[Nm]
            # self.sub_code = self.create_subscription(UInt32, 'key_pressed', self.on_code,10)
            self._command_timer = self.create_timer(1/self.publish_frequency, self.timer_callback)
            self.get_logger().warn("Inspection Configured on mode {}".format(self.mode))  
        # self.get_logger().warn("Inspection Configured with parameter {}".format(self.steering_step))
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        if(self.mode=="bench"): 
            self.sub_code = self.create_subscription(UInt32, 'key_pressed', self.on_code,10)
     
        if(self.mode=="inspection"):
            self._start_time = self.get_time() 
            self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
        
        self._command_timer.cancel()
        self._command_timer.reset()

        self.get_logger().warn(f"\n-- Inspection Activated!")
        return super().on_activate(state)
    
    def on_deactivate(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        
        self.get_logger().warn(f"\n-- Inspection Deactivated!")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state:State) -> TransitionCallbackReturn:
        if(self.mode=="bench"): #albanian solution 
            self._command_timer.cancel()
            self.destroy_timer(self._command_timer)
        self.destroy_publisher(self._command_publisher)
        self.destroy_publisher(self._state_publisher)
        self.destroy_publisher(self._steering_publisher)

        self.destroy_subscription(self._steering_sub)
        self.destroy_subscription(self._motor_sub)
        
        self._steering_angle = 0.0
        self._actual_torque = 0.0

        self.get_logger().warn(f"\n-- Inspection Un-Configured!")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        if (state.state_id == 1):
            self.get_logger().info(f'\n-- Inspection Shutdown!')
            return TransitionCallbackReturn.SUCCESS

        self._command_timer.cancel()
        self.destroy_timer(self._command_timer)

        self.destroy_publisher(self._command_publisher)
        self.destroy_publisher(self._state_publisher)
        self.destroy_publisher(self._steering_publisher)

        self.destroy_subscription(self._steering_sub)
        self.destroy_subscription(self._motor_sub)

        self.get_logger().info(f"\n-- Inspection Shutdown!")
        return TransitionCallbackReturn.SUCCESS
    
    #for mode bench
    def load_from_config(self) -> None:
        # serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value
        # orientation = self.get_parameter('orientation').get_parameter_value().string_value
        self.max_torque = self.get_parameter('max_torque').get_parameter_value().double_value
        self.min_torque = self.get_parameter('min_torque').get_parameter_value().double_value
        self.torque_step = self.get_parameter('torque_step').get_parameter_value().double_value
        self.max_steering = self.get_parameter('max_steering').get_parameter_value().double_value
        self.min_steering = self.get_parameter('min_steering').get_parameter_value().double_value
        self.steering_step = self.get_parameter('steering_step').get_parameter_value().double_value
        self.max_press = self.get_parameter('max_press').get_parameter_value().double_value
        self.min_press = self.get_parameter('min_press').get_parameter_value().double_value
        self.press_step = self.get_parameter('press_step').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.minvel = self.get_parameter('minvel').get_parameter_value().double_value
        self.maxvel = self.get_parameter('maxvel').get_parameter_value().double_value

        self.motor_by_func = self.get_parameter('motor_by_function').get_parameter_value().bool_value
        self.motor_func = self.get_parameter('motor_function').get_parameter_value().string_value
        self.motor_freq = self.get_parameter('motor_frequency').get_parameter_value().double_value
        self.steer_by_func = self.get_parameter('steer_by_function').get_parameter_value().bool_value
        self.steer_func = self.get_parameter('steer_function').get_parameter_value().string_value
        self.steer_freq = self.get_parameter('steer_frequency').get_parameter_value().double_value
        self.brake_by_func = self.get_parameter('brake_by_function').get_parameter_value().bool_value
        self.brake_func = self.get_parameter('brake_function').get_parameter_value().string_value
        self.brake_freq = self.get_parameter('brake_frequency').get_parameter_value().double_value
        
    def timer_callback(self):
        msg = TxControlCommand()
        msg2 = TxSteeringParams()
        if not self.brake_by_func:
            msg.brake_pressure_target = self._brake_command
        elif self.brake_func == 'sine':
            msg.brake_pressure_target = (self.min_press + self.max_press)/2 + 0.5*(self.max_press - self.min_press)*sin(2*pi*self.brake_freq*time.monotonic_ns()*1e-9)
        elif self.brake_func == 'square':
            normd_t = (time.monotonic_ns()*1e-9)%(1/self.brake_freq)
            msg.brake_pressure_target = self.max_press if normd_t<(1/(2*self.brake_freq)) else self.min_press
        
        if not self.steer_by_func:
            msg.steering_angle_target = self._steering_command
        elif self.steer_func == 'sine':
            msg.steering_angle_target = (self.min_steering + self.max_steering)/2 + 0.5*(self.max_steering - self.min_steering)*sin(2*pi*self.steer_freq*time.monotonic_ns()*1e-9)
        elif self.steer_func == 'square':
            normd_t = (time.monotonic_ns()*1e-9)%(1/self.steer_freq)
            msg.steering_angle_target = self.max_steering if normd_t<(1/(2*self.steer_freq)) else self.min_steering
        if not self.motor_by_func:
            msg.motor_torque_target = self._torque_command
        elif self.motor_func == 'sine':
            msg.motor_torque_target = (self.min_torque + self.max_torque)/2 + 0.5*(self.max_torque - self.min_torque)*sin(2*pi*self.motor_freq*time.monotonic_ns()*1e-9)
        elif self.motor_func == 'square':
            normd_t = (time.monotonic_ns()*1e-9)%(1/self.motor_freq)
            msg.motor_torque_target = self.max_torque if normd_t<(1/(2*self.motor_freq)) else self.min_torque

        msg.speed_actual = 0
        msg.speed_target = 0
        self._command_publisher.publish(msg)
        msg2.kp = self.kp
        msg2.kd = self.kd
        msg2.ki = self.ki
        msg2.dt = self.dt
        msg2.minvel = self.minvel
        msg2.maxvel = self.maxvel
        self._steering_publisher.publish(msg2)
    
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
            self.get_logger().info('Increasing motor torque')
            self._torque_command += self.torque_step
            if(self._torque_command>self.max_torque): 
                self._torque_command=self.max_torque
                self.get_logger().warn("Trying to exceed maximum torque set at {} Nm.".format(self.max_torque))
        elif msg.data == keyboard.Key.down.value.vk:
            self.get_logger().info('Decreasing motor torque')
            self._torque_command -= self.torque_step
            if(self._torque_command<self.min_torque): 
                self._torque_command=self.min_torque
                self.get_logger().warn("Trying to exceed minimum torque set at {} Nm.".format(self.min_torque))
        #steering handling
        elif msg.data == keyboard.Key.left.value.vk:
            self.get_logger().info('Steering left')
            self._steering_command -= self.steering_step
            if(self._steering_command<self.min_steering): 
                self._steering_command=self.min_steering
                self.get_logger().warn("Trying to exceed maximum steering rack displ. set at {} mm.".format(self.min_steering))
        elif msg.data == keyboard.Key.right.value.vk:
            self.get_logger().info('Steering right')
            self._steering_command += self.steering_step
            if(self._steering_command>self.max_steering): 
                self._steering_command=self.max_steering
                self.get_logger().warn("Trying to exceed minimum steering rack displ. set at {} mm.".format(self.max_steering))
        #pressure handling
        elif msg.data == keyboard.Key.shift_l.value.vk:
            self.get_logger().info('Decreasing Brake Pressure')
            self._brake_command -= self.press_step
            if(self._brake_command<self.min_press): 
                self._brake_command=self.min_press
                self.get_logger().warn("Trying to exceed minimum brake pressure set at {} bar.".format(self.min_press))
        elif msg.data == keyboard.Key.shift_r.value.vk:
            self.get_logger().info('Increasing Brake Pressure')
            self._brake_command += self.press_step
            if(self._brake_command>self.max_press): 
                self._brake_command=self.max_press
                self.get_logger().warn("Trying to exceed maximum brake pressure set at {} bar.".format(self.max_press))
        elif msg.data == keyboard.Key.caps_lock.value.vk:
            self.get_logger().info('Start sending Mission Finished')
            self.mission_finished = True

            finished_cli = self.create_client(SetTotalLaps, "/p23_status/set_total_laps")
            if not finished_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("P23 Service is not available")
            else:
                req = SetTotalLaps.Request()
                req.total_laps = 0
                self.future = finished_cli.call_async(req)
                self.future.add_done_callback(self.client_callback)
            self.get_logger().info('Sent mission finished')
        #other key pressed
        else:
            self.get_logger().debug('Key ignored: {}'.format(msg.data))

    #for mode inspection   
    def send_commands(self) -> None:
        msg2 = TxSteeringParams()
        msg2.kp = self.kp
        msg2.kd = self.kd
        msg2.ki = self.ki
        msg2.dt = self.dt
        msg2.minvel = self.minvel
        msg2.maxvel = self.maxvel
        self._steering_publisher.publish(msg2)

        if self.mission_finished:
            msg = TxControlCommand()
            msg.brake_pressure_target = 5.0 #value at finish
            self._command_publisher.publish(msg)  
            return

        time = self.get_time() - self._start_time
        msg = TxControlCommand()

        msg.speed_actual = 0
        msg.speed_target = 0
        # Constant torque and sinusoidal movement of the steering wheel
        msg.motor_torque_target = TORQUE_COMMAND
        msg.steering_angle_target = MAX_STEERING * sin(2*pi/STEERING_PERIOD * time)
   
        self._command_publisher.publish(msg)
        
        if time > MISSION_DURATION:
            self.mission_finished = True

            finished_cli = self.create_client(SetTotalLaps, "/p23_status/set_total_laps")
            if not finished_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("P23 Service is not available")
            else:
                req = SetTotalLaps.Request()
                req.total_laps = 0
                self.future = finished_cli.call_async(req)
                self.future.add_done_callback(self.client_callback)
        return

    def set_steering(self, msg:RxSteeringAngle) -> None:
        self._steering_angle = msg.steering_angle
        return

    def set_motor(self, msg:RxVehicleSensors) -> None:
        self._actual_torque = msg.motor_torque_actual
        return

    def get_time(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        return sec + round(nano / 10**9, 3)
    
    def client_callback(self, response):
        if response is not None:
            self.get_logger().warn("Mission Finished")
        else:
            self.get_logger().error("Client call failed")
        return

def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_inspection = InspectionMission()
    executor = SingleThreadedExecutor()
    rclpy.spin(lifecycle_inspection, executor)
    # try:
        
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass
    # finally:
    #     try:
    #         lifecycle_inspection.destroy_node()
    #         rclpy.shutdown()
    #     except Exception:
    #         pass