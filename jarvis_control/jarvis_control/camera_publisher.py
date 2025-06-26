import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Standard ROS 2 message type for images
from cv_bridge import CvBridge     # Package to convert between ROS and OpenCV images
import cv2
import sys

class CameraPublisher(Node):
    """
    This node captures frames from a specific camera using OpenCV and publishes them
    as ROS 2 Image messages.
    """
    def __init__(self):
        # Initialize the node with the name 'camera_publisher'
        super().__init__('camera_publisher')
        
        # --- Parameters ---
        # We discovered from our tests that the USB camera is at index 1!
        self.camera_index = 1
        
        # A standard topic name for camera feeds is 'image_raw'.
        # We can place it under a 'video' namespace for organization.
        self.publish_topic_name = 'video/image_raw'
        
        # We'll aim for about 20 frames per second. 1/20 = 0.05 seconds.
        self.timer_period = 0.05
        
        # Create a publisher for the Image message
        self.publisher_ = self.create_publisher(Image, self.publish_topic_name, 10)
        
        # Create a timer that will call the timer_callback function periodically
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize the video capture object from OpenCV
        self.get_logger().info(f"Attempting to open camera at index {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        
        # Check if the camera was opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera at index {self.camera_index}. Shutting down node Marvin.")
            # Exit if camera cannot be opened on startup.
            # In a more advanced robot, you might want to try reconnecting.
            sys.exit(1)
            
        self.get_logger().info(f"Camera at index {self.camera_index} opened successfully.")
        self.get_logger().info(f"Publishing frames to '{self.publish_topic_name}'.")
        
        # Create a CvBridge object to convert between OpenCV and ROS formats
        self.bridge = CvBridge()

    def timer_callback(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        
        if ret:
            # If a frame was captured successfully, convert the OpenCV image (frame)
            # to a ROS 2 Image message and publish it.
            # The 'bgr8' encoding is standard for color OpenCV images in ROS.
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        else:
            self.get_logger().warn('Could not read frame from camera. Is it disconnected?')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    camera_publisher = None # Define outside of try block
    try:
        # Create the node
        camera_publisher = CameraPublisher()
        # Spin the node so the timer_callback function is called
        rclpy.spin(camera_publisher)
    except SystemExit:
        # This will catch the sys.exit(1) if the camera fails to open
        rclpy.logging.get_logger("camera_publisher_main").info('Camera not found, shutting down.')
    except KeyboardInterrupt:
        # This catches Ctrl+C
        pass
    finally:
        # Cleanly destroy the node and shut down rclpy
        if camera_publisher and rclpy.ok():
            camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

