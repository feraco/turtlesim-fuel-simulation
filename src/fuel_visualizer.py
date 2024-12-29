import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class FuelVisualizer(Node):
    def __init__(self):
        super().__init__('fuel_visualizer')
        self.bridge = CvBridge()
        self.fuel_level = 100.0
        self.image_sub = self.create_subscription(Image, '/turtle1/image', self.image_callback, 10)
        self.fuel_sub = self.create_subscription(Float32, '/turtle1/fuel', self.update_fuel, 10)

    def update_fuel(self, msg):
        self.fuel_level = msg.data

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # Draw fuel level
        fuel_bar_length = int((self.fuel_level / 100.0) * 200)
        cv2.rectangle(frame, (10, 10), (10 + fuel_bar_length, 30), (0, 255, 0), -1)
        cv2.putText(frame, f'Fuel: {int(self.fuel_level)}%', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Publish or display updated image
        cv2.imshow('Turtlesim with Fuel', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FuelVisualizer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

