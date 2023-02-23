import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

import RPi.GPIO as GPIO
import time
import threading

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')

        # Initalize PWM pins
        self.pinA_pwm1 = 13
        self.pinB_pwm1 = 15
        self.pinA_pwm2 = 16
        self.pinB_pwm2 = 18
        self.pinA_pwm3 = 29
        self.pinB_pwm3 = 31
        self.pinA_pwm4 = 32
        self.pinB_pwm4 = 33

        # Setup pins
        GPIO.setup(self.pinA_pwm1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinB_pwm1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinA_pwm2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinB_pwm2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinA_pwm3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinB_pwm3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinA_pwm4, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinB_pwm4, GPIO.OUT, initial=GPIO.LOW)

        # Initialize PWM values
        self.duty_pwm1 = 0
        self.duty_pwm2 = 0
        self.duty_pwm3 = 0
        self.duty_pwm4 = 0

        # PWM period
        self.pwm_period = 1 / 50_000

        # Set up subscribers
        self.pwm1_sub = self.create_subscription(
            Int8,
            '/motors/pwm1',
            self.pwm1_callback,
            10
        )
        self.pwm2_sub = self.create_subscription(
            Int8,
            '/motors/pwm1',
            self.pwm2_callback,
            10
        )
        self.pwm3_sub = self.create_subscription(
            Int8,
            '/motors/pwm1',
            self.pwm3_callback,
            10
        )
        self.pwm4_sub = self.create_subscription(
            Int8,
            '/motors/pwm1',
            self.pwm4_callback,
            10
        )

        # Start threads
        thread1 = threading.Thread(target=self.pwm1_timer, args=(self))
        thread2 = threading.Thread(target=self.pwm2_timer, args=(self))
        thread3 = threading.Thread(target=self.pwm3_timer, args=(self))
        thread4 = threading.Thread(target=self.pwm4_timer, args=(self))\
        
        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()

        thread1.join()
        thread2.join()
        thread3.join()
        thread4.join()

        self.get_logger().info('Motor Controller Initialized.')

    def pwm1_callback(self, msg):
        self.duty_pwm1 = msg

    def pwm2_callback(self, msg):
        self.duty_pwm2 = msg

    def pwm3_callback(self, msg):
        self.duty_pwm3 = msg

    def pwm4_callback(self, msg):
        self.duty_pwm4 = msg
        
    def pwm1_timer(self):
        while(True):
            t_on = self.duty_pwm1 * self.pwm_period
            t_off = self.pwm_period - t_on
            if (self.duty_pwm1 >= 0):
                GPIO.output(self.pinA_pwm1, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinA_pwm1, GPIO.LOW)
                time.sleep(t_off)
            else:
                GPIO.output(self.pinB_pwm1, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinB_pwm1, GPIO.LOW)
                time.sleep(t_off)

            

    def pwm2_timer(self):
        while(True):
            t_on = self.duty_pwm2 * self.pwm_period
            t_off = self.pwm_period - t_on
            if (self.duty_pwm2 >= 0):
                GPIO.output(self.pinA_pwm2, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinA_pwm2, GPIO.LOW)
                time.sleep(t_off)
            else:
                GPIO.output(self.pinB_pwm2, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinB_pwm2, GPIO.LOW)
                time.sleep(t_off)

    def pwm3_timer(self):
        while(True):
            t_on = self.duty_pwm3 * self.pwm_period
            t_off = self.pwm_period - t_on
            if (self.duty_pwm3 >= 0):
                GPIO.output(self.pinA_pwm3, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinA_pwm3, GPIO.LOW)
                time.sleep(t_off)
            else:
                GPIO.output(self.pinB_pwm3, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinB_pwm3, GPIO.LOW)
                time.sleep(t_off)

    def pwm4_timer(self):
        while(True):
            t_on = self.duty_pwm4 * self.pwm_period
            t_off = self.pwm_period - t_on
            if (self.duty_pwm4 >= 0):
                GPIO.output(self.pinA_pwm4, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinA_pwm4, GPIO.LOW)
                time.sleep(t_off)
            else:
                GPIO.output(self.pinB_pwm4, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pinB_pwm4, GPIO.LOW)
                time.sleep(t_off)

    
        
def main(args=None):
    # Initialize Node
    rclpy.init(args=args)

    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)
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