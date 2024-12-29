import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager')

        # Publisher for instructions
        self.instruction_pub = self.create_publisher(String, '/scenario_instructions', 10)

        # Scenario list
        self.scenarios = [
            "Scenario 1: Move the turtle to (5.0, 5.0) with 100 units of fuel. Each movement consumes 10 units of fuel. Stop if fuel runs out.",
            "Scenario 2: Refueling is available at (4.0, 4.0). Start with 50 fuel and reach (8.0, 8.0). Each movement consumes 5 units.",
            "Scenario 3: Optimize fuel usage to visit waypoints: (2.0, 3.0), (6.0, 7.0), and (8.0, 1.0). Forward movement uses 10 units, turns use 5.",
            "Scenario 4: Emergency fuel management. Start at (1.0, 1.0) with 20 fuel and reach (3.0, 3.0) efficiently.",
            "Scenario 5: Avoid dynamic obstacles while reaching (8.0, 8.0). Each avoidance uses 10 units of fuel.",
            "Scenario 6: Collaborative refueling. Two turtles must complete their goals: Turtle1 at (5.0, 5.0) and Turtle2 at (7.0, 2.0)."
        ]

        # Publish the first scenario at startup
        self.current_scenario_index = 0
        self.publish_scenario()

        # Timer to periodically send instructions
        self.timer = self.create_timer(5.0, self.publish_scenario)

    def publish_scenario(self):
        if self.current_scenario_index < len(self.scenarios):
            msg = String()
            msg.data = self.scenarios[self.current_scenario_index]
            self.instruction_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
        else:
            self.get_logger().info("All scenarios completed!")

    def next_scenario(self):
        """Move to the next scenario."""
        if self.current_scenario_index < len(self.scenarios) - 1:
            self.current_scenario_index += 1
            self.publish_scenario()

def main(args=None):
    rclpy.init(args=args)
    scenario_manager = ScenarioManager()
    rclpy.spin(scenario_manager)
    scenario_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
