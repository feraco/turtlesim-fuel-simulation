import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading

class InteractiveFuelScenario(Node):
    def __init__(self):
        super().__init__('interactive_fuel_scenario')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.fuel = 100  # Initial fuel
        self.goal = (5.0, 5.0)
        self.pose = None
        self.running = True

    def pose_callback(self, msg):
        self.pose = (msg.x, msg.y)

    def start_interactive_session(self):
        print("\n--- Turtlesim Fuel Challenge ---")
        print("Instructions:")
        print("1. Move the turtle to (5.0, 5.0) using the command interface.")
        print("2. Each forward movement consumes 10 units of fuel.")
        print("3. Stop the turtle if fuel runs out.\n")

        while self.running and self.fuel > 0:
            if self.pose:
                if (abs(self.pose[0] - self.goal[0]) < 0.5 and
                        abs(self.pose[1] - self.goal[1]) < 0.5):
                    print("Congratulations! You reached the goal!")
                    break

            print(f"Current Fuel: {self.fuel}")
            print(f"Current Position: {self.pose if self.pose else 'N/A'}")
            print("Enter movement commands (linear_velocity angular_velocity): ")
            try:
                user_input = input("> ").strip()
                if user_input.lower() in ["exit", "quit"]:
                    print("Exiting scenario...")
                    self.running = False
                    break

                linear, angular = map(float, user_input.split())
                self.execute_movement(linear, angular)
            except ValueError:
                print("Invalid input. Please enter two numbers separated by a space.")

    def execute_movement(self, linear, angular):
        if self.fuel <= 0:
            print("Out of fuel! The turtle cannot move.")
            return

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

        # Simulate fuel consumption
        self.fuel -= abs(linear) * 10 + abs(angular) * 5
        if self.fuel <= 0:
            print("The turtle ran out of fuel! Stopping.")
            self.publisher.publish(Twist())  # Stop the turtle

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveFuelScenario()

    # Run the interactive session in a separate thread to keep ROS 2 spinning
    threading.Thread(target=node.start_interactive_session, daemon=True).start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
