import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

import serial

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')

        # Set up serial ports
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)

        # Set up subscribers
        self.steering_sub = self.create_subscription(
            Int8,
            '/motors/steering_angle',
            self.steering_callback,
            10
        )
        self.drive_sub = self.create_subscription(
            Int8,
            '/motors/drive',
            self.drive_callback,
            10
        )

        #test
        self.serial_port.write(52)

        self.get_logger().info('Motor Controller Initialized.')

    def steering_callback(self, msg):
        self.steering_angle = msg;

    def drive_callback(self, msg):
        self.drive_power = msg;
    
        
def main(args=None):
    # Initialize Node
    rclpy.init(args=args)

    # Initialize Motor Controller
    motor_controller_node = MotorControllerNode()

    # Spin node
    rclpy.spin(motor_controller_node)

    # Destroy when finished
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
