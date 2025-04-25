import rclpy
from rclpy.node import Node
from robot_pwm_msgs.msg import MotorPwm
import pygame
import threading
import time

class PS5Publisher(Node):
    def __init__(self):
        super().__init__('ps5_publisher')
        print("DEBUG: PS5Publisher node initializing...")

        self.publisher_ = self.create_publisher(MotorPwm, 'motor_pwm', 10)
        print("DEBUG: Publisher created")

        self.timer = self.create_timer(0.05, self.publish_pwm)
        print("DEBUG: Timer created")

        pygame.init()
        pygame.joystick.init()
        print("DEBUG: Pygame initialized")

        while pygame.joystick.get_count() == 0:
            print("DEBUG: Waiting for PS5 controller...")
            pygame.joystick.quit()
            pygame.joystick.init()
            time.sleep(2)

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"DEBUG: Connected to: {self.controller.get_name()}")

        self.deadzone = 0.13
        self.max_speed = 3000
        self.target_FL = 0
        self.target_FR = 0
        self.target_BL = 0
        self.target_BR = 0
        self.lock = threading.Lock()

        self.input_thread = threading.Thread(target=self.controller_loop, daemon=True)
        self.input_thread.start()
        print("DEBUG: Controller loop started")

    def scale_input(self, value):
        return 0 if abs(value) < self.deadzone else int(value * self.max_speed)

    def controller_loop(self):
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.update_speeds()
            time.sleep(0.01)

    def update_speeds(self):
        lx = self.scale_input(self.controller.get_axis(0))
        ly = self.scale_input(-self.controller.get_axis(1))
        rx = self.scale_input(-self.controller.get_axis(3))

        with self.lock:
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
        print(f"FL: {msg.speed_fl}, FR: {msg.speed_fr}, BL: {msg.speed_bl}, BR: {msg.speed_br}")

def main(args=None):
    print("DEBUG: Entering main()")
    rclpy.init(args=args)
    node = PS5Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

