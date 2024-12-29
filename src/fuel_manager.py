import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.srv import TeleportAbsolute

class FuelManager(Node):
    def __init__(self):
        super().__init__('fuel_manager')
        self.fuel_level = 100.0  # Initial fuel level
        self.publisher = self.create_publisher(Float32, '/turtle1/fuel', 10)
        self.timer = self.create_timer(0.1, self.publish_fuel_level)
        self.create_subscription(Float32, '/turtle1/fuel_decrease', self.decrease_fuel, 10)

    def publish_fuel_level(self):
        msg = Float32()
        msg.data = self.fuel_level
        self.publisher.publish(msg)

    def decrease_fuel(self, msg):
        self.fuel_level -= msg.data
        if self.fuel_level <= 0:
            self.get_logger().info('Out of fuel! Resetting turtle...')
            self.reset_turtle()

    def reset_turtle(self):
        self.fuel_level = 100.0
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        request = TeleportAbsolute.Request()
        request.x = 5.0
        request.y = 5.0
        client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = FuelManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
