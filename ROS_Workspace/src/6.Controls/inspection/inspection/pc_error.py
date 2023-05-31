import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_msgs.msg import TxSystemState, DriverlessStatus

class SendError(Node):
    '''
    Node for electronics to test their pc error handling
    '''
    def __init__(self) -> None:
        super().__init__('test_on_error')
        self._pub = self.create_publisher(TxSystemState, '/system_state', 10)


def main(args=None) -> None:
    rclpy.init(args=args)

    error_node = rclpy.create_node('test_on_error')
    pub = error_node.create_publisher(TxSystemState, '/system_state', 10)
    
    executor = SingleThreadedExecutor()

    send = True
    try:
        while (rclpy.ok):
            msg = TxSystemState()
            msg.dv_status.id = DriverlessStatus.NODE_PROBLEM
            if send:            
                pub.publish(msg)
                send = False
            rclpy.spin_once(error_node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        error_node.destroy_node()
        rclpy.shutdown()