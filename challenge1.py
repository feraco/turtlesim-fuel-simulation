import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class MoveWithFuel(Node):
    def __init__(self):
        super().__init__('move_with_fuel')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.fuel = 100
        self.goal = (5.0, 5.0)
        self.pose = None
        self.initialize_goal_marker()

    def initialize_goal_marker(self):
        # Set a red marker for the goal
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_pen service...')
        request = SetPen.Request()
        request.r = 255
        request.g = 0
        request.b = 0
        request.width = 5
        client.call_async(request)

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)
        if self.fuel <= 0:
            self.get_logger().info("Out of fuel! Stopping.")
            self.publisher.publish(Twist())  # Stop the turtle
        elif abs(self.pose[0] - self.goal[0]) < 0.5 and abs(self.pose[1] - self.goal[1]) < 0.5:
            self.get_logger().info("Congratulations! You reached the goal!")

    def move_turtle(self, linear, angular):
        if self.fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            self.fuel -= abs(linear) * 10 + abs(angular) * 5
            self.get_logger().info(f"Fuel remaining: {self.fuel}")
        else:
            self.get_logger().info("No fuel remaining!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveWithFuel()
    print("Goal: Move the turtle to (5.0, 5.0) with 100 fuel units.")
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
