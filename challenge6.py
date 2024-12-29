import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class CollaborativeRefueling(Node):
    def __init__(self):
        super().__init__('collaborative_refueling')
        self.t1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.t2_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.t1_sub = self.create_subscription(Pose, '/turtle1/pose', self.t1_pose_callback, 10)
        self.t2_sub = self.create_subscription(Pose, '/turtle2/pose', self.t2_pose_callback, 10)
        self.t1_fuel = 30
        self.t2_fuel = 50
        self.t1_pose = None
        self.t2_pose = None
        self.t1_goal = (5.0, 5.0)
        self.t2_goal = (7.0, 2.0)

    def t1_pose_callback(self, msg):
        self.t1_pose = (msg.x, msg.y)
        if math.sqrt((self.t1_pose[0] - self.t1_goal[0])**2 + (self.t1_pose[1] - self.t1_goal[1])**2) < 0.5:
            self.get_logger().info("Turtle 1 reached the goal!")

    def t2_pose_callback(self, msg):
        self.t2_pose = (msg.x, msg.y)
        if math.sqrt((self.t2_pose[0] - self.t2_goal[0])**2 + (self.t2_pose[1] - self.t2_goal[1])**2) < 0.5:
            self.get_logger().info("Turtle 2 reached the goal!")

    def move_turtle(self, turtle, linear, angular):
        if turtle == 1 and self.t1_fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.t1_pub.publish(twist)
            self.t1_fuel -= abs(linear) * 5 + abs(angular) * 2
            self.get_logger().info(f"Turtle 1 Fuel remaining: {self.t1_fuel}")
        elif turtle == 2 and self.t2_fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.t2_pub.publish(twist)
            self.t2_fuel -= abs(linear) * 5 + abs(angular) * 2
            self.get_logger().info(f"Turtle 2 Fuel remaining: {self.t2_fuel}")
        else:
            self.get_logger().info(f"Turtle {turtle} is out of fuel!")

    def transfer_fuel(self, amount):
        if self.t1_pose and self.t2_pose:
            dist = math.sqrt((self.t1_pose[0] - self.t2_pose[0])**2 + (self.t1_pose[1] - self.t2_pose[1])**2)
            if dist < 1.0:
                if self.t2_fuel >= amount:
                    self.t2_fuel -= amount
                    self.t1_fuel += amount
                    self.get_logger().info(f"Transferred {amount} fuel from Turtle 2 to Turtle 1.")
                else:
                    self.get_logger().info("Not enough fuel to transfer.")

def main(args=None):
    rclpy.init(args=args)
    node = CollaborativeRefueling()
    print("Goal: Help Turtle 1 reach (5.0, 5.0) and Turtle 2 reach (7.0, 2.0).")
    print("Commands: ")
    print("- Move Turtle 1: '1 linear angular'")
    print("- Move Turtle 2: '2 linear angular'")
    print("- Transfer Fuel: 'transfer amount'")
    while rclpy.ok():
        try:
            user_input = input("> ").strip()
            if user_input.lower() in ['exit', 'quit']:
                break
            elif user_input.startswith("1") or user_input.startswith("2"):
                turtle, linear, angular = map(float, user_input.split())
                node.move_turtle(int(turtle), linear, angular)
            elif user_input.startswith("transfer"):
                _, amount = user_input.split()
                node.transfer_fuel(int(amount))
        except ValueError:
            print("Invalid input. Use 'turtle linear angular' or 'transfer amount'.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
