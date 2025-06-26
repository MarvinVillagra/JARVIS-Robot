import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm
from adafruit_pca9685 import PCA9685
# This is the correct, direct way to initialize I2C on an unsupported board
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as BlinkaI2C

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info("Initializing I2C bus for Motor Controller...")
        try:
            # Explicitly initialize I2C bus #2, bypassing board detection
            i2c = BlinkaI2C(2)
            # Initialize the PCA9685 with this specific I2C object
            self.pca = PCA9685(i2c, address=0x40) # Specify the address for clarity
            self.pca.frequency = 1000
            self.get_logger().info("PCA9685 motor driver found and initialized on I2C bus 2.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C or PCA9685: {e}")
            raise e
        # This map defines which PCA9685 channels control which part of the motor driver
        self.motor_map = {
            'fl': {'pwm': 0, 'in1': 1, 'in2': 2}, # Front-Left motor
            'fr': {'pwm': 3, 'in1': 4, 'in2': 5}, # Front-Right motor
            'br': {'pwm': 6, 'in1': 7, 'in2': 8}, # Back-Right motor
            'bl': {'pwm': 9, 'in1': 10, 'in2': 11}, # Back-Left motor
        }

        # Create a subscriber to listen for motor commands on the 'motor_pwm' topic
        self.subscription = self.create_subscription(
            MotorPwm,
            'motor_pwm',
            self.motor_callback,
            10
        )
        self.get_logger().info("Motor Controller node is ready and waiting for commands on 'motor_pwm' topic.")

    def set_motor(self, label, speed):
        """Sets the speed and direction for an individual motor."""
        pins = self.motor_map[label]
        
        # The PCA9685 has a 12-bit resolution, so values range from 0 to 4095.
        # We clamp the absolute speed to this range.
        duty_cycle = max(0, min(abs(speed), 4095))

        # Set the direction pins (IN1, IN2) on the motor driver
        if speed > 0: # Forward
            self.pca.channels[pins['in1']].duty_cycle = 0xFFFF # ON
            self.pca.channels[pins['in2']].duty_cycle = 0x0000 # OFF
        elif speed < 0: # Reverse
            self.pca.channels[pins['in1']].duty_cycle = 0x0000 # OFF
            self.pca.channels[pins['in2']].duty_cycle = 0xFFFF # ON
        else: # Brake/Stop
            self.pca.channels[pins['in1']].duty_cycle = 0x0000 # OFF
            self.pca.channels[pins['in2']].duty_cycle = 0x0000 # OFF

        # Set the speed via the PWM duty cycle.
        # The library expects a 16-bit value, so we scale our 12-bit value up.
        self.pca.channels[pins['pwm']].duty_cycle = duty_cycle * 16

    def motor_callback(self, msg):
        """This function is called every time a message is received on the topic."""
        # self.get_logger().info(f"Received speeds: FL={msg.fl}, FR={msg.fr}, BL={msg.bl}, BR={msg.br}")
        self.set_motor('fl', msg.fl)
        self.set_motor('fr', msg.fr)
        self.set_motor('bl', msg.bl)
        self.set_motor('br', msg.br)

def main(args=None):
    rclpy.init(args=args)
    
    motor_controller_node = None
    try:
        motor_controller_node = MotorController()
        rclpy.spin(motor_controller_node)
    except Exception as e:
        rclpy.logging.get_logger("motor_controller_main").error(f"Node failed to start or encountered an error: {e}")
    finally:
        # Cleanly destroy the node and shut down rclpy on exit
        if motor_controller_node and rclpy.ok():
            motor_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

