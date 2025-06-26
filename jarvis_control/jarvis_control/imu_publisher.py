import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
# This is the correct, direct way to initialize I2C
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as BlinkaI2C
# This is the correct module to import for your sensor
import adafruit_lsm6ds

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.get_logger().info("Initializing I2C bus for IMU Publisher...")
        try:
            # Explicitly initialize I2C bus #2
            self.i2c = BlinkaI2C(2)
            # Initialize the sensor with the specific I2C object and address
            self.sensor = adafruit_lsm6ds.LSM6DSOX(self.i2c, address=0x68)
            self.get_logger().info("IMU sensor LSM6DSOX found and initialized on I2C bus 2.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C or find IMU sensor: {e}")
            raise e

        # Create a publisher for the Imu message
        self.publisher_ = self.create_publisher(Imu, self.publish_topic_name, 10)

        # Create a timer that calls the publishing function periodically
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

        self.get_logger().info(f"IMU Publisher node started. Publishing to '{self.publish_topic_name}' at {1/self.timer_period:.1f} Hz.")

    def publish_imu_data(self):
        # Create an Imu message object
        msg = Imu()

        # Populate the message header with current time and frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation: This IMU does not provide a fused orientation (quaternion).
        # We publish a zero-orientation quaternion and set its covariance to -1
        # to indicate that this data is not provided by this sensor.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0 # A valid quaternion must not be all zeros. w=1 represents no rotation.
        msg.orientation_covariance[0] = -1.0

        # Angular Velocity (Gyroscope)
        # The sensor library provides data in radians/sec, which is what ROS expects.
        gyro_x, gyro_y, gyro_z = self.sensor.gyro
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance[0] = -1.0 # Mark covariance as unknown

        # Linear Acceleration
        # The sensor library provides data in m/s^2, which is what ROS expects.
        accel_x, accel_y, accel_z = self.sensor.acceleration
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.linear_acceleration_covariance[0] = -1.0 # Mark covariance as unknown

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    imu_publisher = None
    try:
        imu_publisher = ImuPublisher()
        rclpy.spin(imu_publisher)
    except Exception as e:
        rclpy.logging.get_logger("imu_publisher_main").error(f"Node failed to start or encountered an error: {e}")
    finally:
        # Cleanly destroy the node and shut down rclpy on exit
        if imu_publisher and rclpy.ok():
            imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
