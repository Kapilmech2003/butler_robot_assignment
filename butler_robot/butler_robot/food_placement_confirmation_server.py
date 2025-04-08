import rclpy
from rclpy.node import Node
from butler_bot_interfaces.srv import ConfirmArrival

class ConfirmationServer(Node):
    def __init__(self):
        super().__init__('confirmation_server')
        self.srv = self.create_service(ConfirmArrival, 'confirm_arrival', self.handle_confirm_request)

    def handle_confirm_request(self, request, response):
        self.get_logger().info(f"Waiting for confirmation at {request.location}...")
        confirmation = input(f"Confirm arrival at {request.location} (yes/no): ").strip().lower()

        response.confirmed = confirmation == 'yes'
        if response.confirmed:
            self.get_logger().info(f"Arrival at {request.location} confirmed.")
        else:
            self.get_logger().info(f"Arrival at {request.location} NOT confirmed.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConfirmationServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
