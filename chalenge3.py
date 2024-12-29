import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class OptimizeFuelUsage(Node):
    def __init__(self):
        super().__init__('optimize_fuel_usage')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.fuel = 100
        self.pose = None
        self.waypoints = [(2.0, 3.0), (6.0, 7.0), (8.0, 1.0)]
        self.current_waypoint_index = 0

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)
        if self.current_waypoint_index < len(self.waypoints):
            goal = self.waypoints[self.current_waypoint_index]
            if math.sqrt((self.pose[0] - goal[0])**2 + (self.pose[1] - goal[1])**2) < 0.5:
                self.get_logger().info(f"Reached waypoint {goal}")
                self.current_waypoint_index += 1
                if self.current_waypoint_index == len(self.waypoints):
                    self.get_logger().info("Congratulations! All waypoints visited!")

    def move_turtle(self, linear, angular):
        if self.fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            self.fuel -= abs(linear) * 10 + abs(angular) * 5
            self.get_logger().info(f"Fuel remaining: {self.fuel}")
        else:
            self.get_logger().info("Out of fuel!")

def main(args=None):
    rclpy.init(args=args)
    node = OptimizeFuelUsage()
    print("Goal: Visit waypoints (2.0, 3.0), (6.0, 7.0), and (8.0, 1.0) with 100 units of fuel.")
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
