import sys

# pynput throws an error if we import it before $DISPLAY is set on LINUX
from pynput.keyboard import KeyCode
import os
from pynput import keyboard
import rclpy
from rclpy.parameter import Parameter
import std_msgs.msg


class KeystrokeListen:
    def __init__(self, name=None):
        self.node = rclpy.create_node(name or type(self).__name__)
        self.pub_code = self.node.create_publisher(std_msgs.msg.UInt32, 'key_pressed',qos_profile=10)
        if self.exit_on_esc:
            self.logger.info('To end this node, press the escape key')

    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)

    @property
    def logger(self):
        return self.node.get_logger()

    @property
    def exit_on_esc(self):
        param = self.node.declare_parameter('exit_on_esc')

        if param.type_ != Parameter.Type.BOOL:
            new_param = Parameter('exit_on_esc', Parameter.Type.BOOL, True)
            self.logger.warn(
                'Parameter {}={} is a {} but expected a boolean. Assuming {}.'.format(param.name,
                                                                                      param.value,
                                                                                      param.type_,
                                                                                      new_param.value),
                once=True)
            self.node.set_parameters([new_param])
            param = new_param

        value = param.value
        assert isinstance(value, bool)
        return value

    def on_release(self, key):
        #dont need to do something at release since we are sending on press
        pass

    def on_press(self, key):
        try:
            char = getattr(key, 'char', None)
            if isinstance(char, str):
                self.logger.info('pressed ' + char)
            else:
                try:
                    # known keys like spacebar, ctrl
                    name = key.name
                    vk = key.value.vk
                except AttributeError:
                    # unknown keys like headphones skip song button
                    name = 'UNKNOWN'
                    vk = key.vk
                self.logger.info('pressed {} ({})'.format(name, vk))
                self.pub_code.publish(self.pub_code.msg_type(data=vk))
        except Exception as e:
            self.logger.error(str(e))
            raise

        if key == keyboard.Key.esc and self.exit_on_esc:
            self.logger.info('stopping listener')
            raise keyboard.Listener.StopException


def main(args=None):
    rclpy.init(args=args)
    KeystrokeListen().spin()

if __name__ == '__main__':
    main()