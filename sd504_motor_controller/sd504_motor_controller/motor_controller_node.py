import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

import RPi.GPIO as GPIO
import time

output_pin_A1 = 15

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Motor Controller Initialized.')
        
def main(args=None):
    # Initialize Node
    rclpy.init(args=args)
    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(output_pin_A1, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setwarnings(False)
    # Initialize Motor Controller
    motor_controller_node = MotorControllerNode()
    # Spin node
    rclpy.spin(motor_controller_node)
    # Destroy when finished
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()