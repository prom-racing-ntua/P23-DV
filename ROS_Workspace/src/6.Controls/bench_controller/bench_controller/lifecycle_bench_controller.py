import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_msgs.msg import String, UInt32
from pynput import keyboard
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
import time
from custom_msgs.msg import *

class BenchController(Node):
    def __init__(self,name=None) -> None:
        self.node = rclpy.create_node(name or type(self).__name__)
        self._steering_command = self.declare_parameter('steering', 0.0).value  #[mm]
        self._brake_command = self.declare_parameter('brake', 0.0).value        #[bar]
        self._torque_command = self.declare_parameter('torque', 0.0).value      #[Nm]
        self.get_logger().warn(f'\n-- Bench controller node Created')
        
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
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.load_from_config()
        
        self.saltas_clock.cancel()
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
    
    def on_cleanup(self, state:State) -> TransitionCallbackReturn:
    
    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        
    
    
    