import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class RefuelingStation(Node):
    def __init__(self):
        super().__init__('refueling_station')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.fuel = 50
        self.goal = (8.0, 8.0)
        self.refuel_station = (4.0, 4.0)
        self.pose = None

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)
        if abs(self.pose[0] - self.refuel_station[0]) < 0.5 and abs(self.pose[1] - self.refuel_station[1]) < 0.5:
            self.fuel += 30
            self.get_logger().info("Refueled! Current fuel: {}".format(self.fuel))

    def move_turtle(self, linear, angular):
        if self.fuel > 0:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            self.fuel -= abs(linear) * 5 + abs(angular) * 2
            self.get_logger().info(f"Fuel remaining: {self.fuel}")
        else:
            self.get_logger().info("No fuel remaining!")

def main(args=None):
    rclpy.init(args=args)
    node = RefuelingStation()
    print("Goal: Move the turtle to (8.0, 8.0) with refueling at (4.0, 4.0).")
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
