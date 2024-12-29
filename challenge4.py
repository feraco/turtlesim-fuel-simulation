import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class EmergencyFuelManagement(Node):
    def __init__(self):
        super().__init__('emergency_fuel_management')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.fuel = 20
        self.pose = None
        self.refuel_station = (3.0, 3.0)

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)
        if math.sqrt((self.pose[0] - self.refuel_station[0])**2 + (self.pose[1] - self.refuel_station[1])**2) < 0.5:
            self.get_logger().info("Refueling successful! You made it!")
            self.fuel += 30  # Refill fuel
            self.get_logger().info(f"New fuel level: {self.fuel}")

    def move_turtle(self, linear, angular):
        if self.fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            self.fuel -= abs(linear) * 10 + abs(angular) * 5
            self.get_logger().info(f"Fuel remaining: {self.fuel}")
        else:
            self.get_logger().info("Out of fuel! You didn't make it.")

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyFuelManagement()
    print("Goal: Reach the refueling station at (3.0, 3.0) with 20 units of fuel.")
    print("Use 'linear angular' commands to move (e.g., '1.0 0.0').")
    while rclpy.ok():
        try:
            user_input = input("> ").strip()
            if user_input.lower() in ['exit', 'quit']:
                break
            linear, angular = map(float, user_input.split())
            node.move_turtle(linear, angular)
        except ValueError:
            print("Invalid input. Use 'linear angular' format.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
