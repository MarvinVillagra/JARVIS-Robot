import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        self.publisher_ = self.create_publisher(MotorPwm, 'motor_pwm', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = 0

    def timer_callback(self):
        msg = MotorPwm()
        msg.speed_fl = self.speed
        msg.speed_fr = self.speed
        msg.speed_bl = self.speed
        msg.speed_br = self.speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        self.speed = (self.speed + 50) % 255

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

