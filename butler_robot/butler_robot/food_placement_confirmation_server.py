import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class ConfirmationServer(Node):
    def __init__(self):
        """ROS 2 node for handling user confirmation input."""
        super().__init__('confirmation_server')
        self.subscription = self.create_subscription(
            String, 'confirmation_request', self.request_callback, 10)
        self.publisher = self.create_publisher(String, 'confirmation_response', 10)

    def request_callback(self, msg):
        """Handles incoming confirmation requests and sends user input back."""
        data = msg.data.split(",")
        location = data[0]
        timeout = int(data[1])

        self.get_logger().info(f"Received confirmation request for {location} (Timeout: {timeout} sec)")

        confirmation = None

        def get_input():
            nonlocal confirmation
            confirmation = input(f"Confirm {location}? (yes/no): ").strip().lower()

        input_thread = threading.Thread(target=get_input)
        input_thread.daemon = True
        input_thread.start()

        input_thread.join(timeout)

        response_msg = String()
        if confirmation == "yes":
            response_msg.data = "yes"
        else:
            response_msg.data = "no"

        self.publisher.publish(response_msg)
        self.get_logger().info(f"Sent response: {response_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ConfirmationServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
