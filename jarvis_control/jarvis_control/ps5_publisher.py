import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm
import pygame
import threading
import time

class PS5Publisher(Node):
    def __init__(self):
        super().__init__('ps5_publisher')
        rclpy.logging.set_logger_level('ps5_publisher', rclpy.logging.LoggingSeverity.INFO)

        self.publisher_ = self.create_publisher(MotorPwm, 'motor_pwm', 10)
        self.timer = self.create_timer(0.05, self.publish_pwm)

        pygame.init()
        pygame.joystick.init()

        # Wait until a controller is detected
        while pygame.joystick.get_count() == 0:
            self.get_logger().warn("Waiting for PS5 controller...")
            pygame.joystick.quit()
            pygame.joystick.init()
            time.sleep(2)

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Connected to: {self.controller.get_name()}")

        self.deadzone = 0.13
        self.max_speed = 3000
        self.target_FL = 0
        self.target_FR = 0
        self.target_BL = 0
        self.target_BR = 0
        self.lock = threading.Lock()

        self.input_thread = threading.Thread(target=self.controller_loop, daemon=True)
        self.input_thread.start()

    def scale_input(self, value):
        return 0 if abs(value) < self.deadzone else int(value * self.max_speed)

    def controller_loop(self):
        while rclpy.ok():
            try:
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        self.update_speeds()
            except pygame.error:
                self.get_logger().warn("Controller disconnected. Waiting for reconnection...")
                self.reconnect_controller()

            time.sleep(0.01)

    def reconnect_controller(self):
        self.controller.quit()
        pygame.joystick.quit()
        time.sleep(1)

        while pygame.joystick.get_count() == 0:
            pygame.joystick.quit()
            pygame.joystick.init()
            time.sleep(2)
            self.get_logger().warn("Still waiting for controller...")

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Reconnected to: {self.controller.get_name()}")

    def update_speeds(self):
        rx = self.scale_input(self.controller.get_axis(0))          # Left stick X = turn
        ly = self.scale_input(-self.controller.get_axis(1))         # Left stick Y = forward/back
        lx = self.scale_input(self.controller.get_axis(3))          # Right stick X = crab walk

        with self.lock:
            if lx == 0 and ly == 0 and rx == 0:
                self.target_FL = 0
                self.target_FR = 0
                self.target_BL = 0
                self.target_BR = 0
            else:
                self.target_FL = ly + lx - rx
                self.target_FR = ly - lx + rx
                self.target_BL = ly - lx - rx
                self.target_BR = ly + lx + rx

    def publish_pwm(self):
        with self.lock:
            msg = MotorPwm()
            msg.speed_fl = self.target_FL
            msg.speed_fr = self.target_FR
            msg.speed_bl = self.target_BL
            msg.speed_br = self.target_BR
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PS5Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

