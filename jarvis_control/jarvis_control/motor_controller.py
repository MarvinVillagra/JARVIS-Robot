import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm

from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Setup PCA9685
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # 1kHz PWM frequency

        self.motor_map = {
            'fl': {'pwm': 0, 'in1': 1, 'in2': 2},
            'fr': {'pwm': 3, 'in1': 4, 'in2': 5},
            'br': {'pwm': 6, 'in1': 7, 'in2': 8},
            'bl': {'pwm': 9, 'in1': 10, 'in2': 11},
        }

        self.subscription = self.create_subscription(
            MotorPwm,
            'motor_pwm',
            self.motor_callback,
            10
        )

    def set_motor(self, label, speed):
        pins = self.motor_map[label]
        abs_speed = min(abs(speed), 255)
        duty_cycle = int((abs_speed / 255) * 0xFFFF)

        if speed > 0:
            self.pca.channels[pins['in1']].duty_cycle = 0xFFFF
            self.pca.channels[pins['in2']].duty_cycle = 0
        elif speed < 0:
            self.pca.channels[pins['in1']].duty_cycle = 0
            self.pca.channels[pins['in2']].duty_cycle = 0xFFFF
        else:
            self.pca.channels[pins['in1']].duty_cycle = 0
            self.pca.channels[pins['in2']].duty_cycle = 0

        self.pca.channels[pins['pwm']].duty_cycle = duty_cycle
        self.get_logger().info(f"Set {label.upper()} motor to {speed}")

    def motor_callback(self, msg):
        self.set_motor('fl', msg.fl)
        self.set_motor('fr', msg.fr)
        self.set_motor('bl', msg.bl)
        self.set_motor('br', msg.br)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

