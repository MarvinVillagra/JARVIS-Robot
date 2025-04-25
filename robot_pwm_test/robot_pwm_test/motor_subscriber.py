import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            MotorPwm,
            'motor_pwm',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

