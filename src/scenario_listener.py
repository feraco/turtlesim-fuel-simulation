import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ScenarioListener(Node):
    def __init__(self):
        super().__init__('scenario_listener')
        self.create_subscription(String, '/scenario_instructions', self.display_instruction, 10)

    def display_instruction(self, msg):
        print(f"*** Current Scenario ***\n{msg.data}\n")

def main(args=None):
    rclpy.init(args=args)
    listener = ScenarioListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
