import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.obstacle_subscription = self.create_subscription(Pose, '/turtle2/pose', self.obstacle_callback, 10)
        self.fuel = 50
        self.pose = None
        self.obstacle_pose = None
        self.goal = (8.0, 8.0)

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)
        if math.sqrt((self.pose[0] - self.goal[0])**2 + (self.pose[1] - self.goal[1])**2) < 0.5:
            self.get_logger().info("Congratulations! You reached the goal!")

    def obstacle_callback(self, msg):
        self.obstacle_pose = (msg.x, msg.y)

    def move_turtle(self, linear, angular):
        if self.fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            self.fuel -= abs(linear) * 5 + abs(angular) * 5

            if self.obstacle_pose and self.pose:
                dist_to_obstacle = math.sqrt((self.pose[0] - self.obstacle_pose[0])**2 + (self.pose[1] - self.obstacle_pose[1])**2)
                if dist_to_obstacle < 1.0:
                    self.get_logger().info("Obstacle detected! Avoiding...")
                    self.fuel -= 10
                    twist.linear.x = 0.0
                    twist.angular.z = 1.0
                    self.publisher.publish(twist)

            self.get_logger().info(f"Fuel remaining: {self.fuel}")
        else:
            self.get_logger().info("Out of fuel!")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacles()
    print("Goal: Reach (8.0, 8.0) with 50 fuel units while avoiding a dynamic obstacle.")
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
