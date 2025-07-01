import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from math import pi
from .mpu_driver import MPU6050

G_TO_MS2 = 9.80665
DEG_TO_RAD = pi / 180.0

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.sensor = MPU6050()
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.05, self.publish_imu_data)

    def publish_imu_data(self):
        sensor_data = self.sensor.get_sensor_data()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = 1.0
        msg.angular_velocity.x = sensor_data['gyro_x'] * DEG_TO_RAD
        msg.angular_velocity.y = sensor_data['gyro_y'] * DEG_TO_RAD
        msg.angular_velocity.z = sensor_data['gyro_z'] * DEG_TO_RAD
        msg.linear_acceleration.x = sensor_data['accel_x'] * G_TO_MS2
        msg.linear_acceleration.y = sensor_data['accel_y'] * G_TO_MS2
        msg.linear_acceleration.z = sensor_data['accel_z'] * G_TO_MS2
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.sensor.close()
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
