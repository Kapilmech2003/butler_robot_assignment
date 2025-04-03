import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.client.wait_for_server()

        # Define locations
        self.locations = {
            "kitchen": (4.28, -0.67, 0.7071, 0.7071),
            "table1": (3.40, 3.48, 0.7071, 0.7071),
            "table2": (-3.11, 3.38, -0.99, -0.03),
            "table3": (-2.88, -0.79, -0.99, 0.12),
            "home": (0.0, 0.0, 0.0, 1.57)
        }

    def send_goal(self, location):
        """Send goal to the given location and wait for success."""
        if location not in self.locations:
            self.get_logger().error(f"Invalid location: {location}")
            return False

        x, y, z, w = self.locations[location]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f'Sending goal to {location}: x={x}, y={y}')
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        
        self.get_logger().info(f"Goal to {location} accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Goal to {location} reached successfully!")
            return True
        else:
            self.get_logger().error(f"Goal to {location} failed with status: {result.status}")
            return False

    def wait_for_confirmation(self, location, timeout=30):
        """Waits for user confirmation at the given location with a timeout."""
        self.get_logger().info(f"Waiting for confirmation at {location} (Timeout: {timeout} sec)...")

        start_time = time.time()
        while time.time() - start_time < timeout:
            confirmation = input(f"Confirm {location}? (yes/no): ").strip().lower()
            if confirmation == "yes":
                self.get_logger().info(f"Confirmed at {location}, proceeding...")
                return True
            elif confirmation == "no":
                self.get_logger().info(f"Not confirmed at {location}.")
                return False

        self.get_logger().info(f"Timeout reached at {location}, skipping to next step.")
        return False

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()

    try:
        while True:
            # Get multiple table numbers as input
            table_input = input("Enter table numbers (e.g., '1 2 3') or press Ctrl+C to exit: ").strip()
            table_numbers = table_input.split()

            # Validate table numbers
            table_keys = []
            for num in table_numbers:
                if f"table{num}" in node.locations:
                    table_keys.append(f"table{num}")
                else:
                    node.get_logger().error(f"Invalid table number: {num}. Please enter 1, 2, or 3.")
                    continue

            if not table_keys:
                node.get_logger().error("No valid tables selected. Try again.")
                continue

            # Navigate: Home → Kitchen → Multiple Tables → Kitchen (if any cancel) → Home
            if node.send_goal("kitchen"):
                if not node.wait_for_confirmation("kitchen"):
                    node.get_logger().info("No confirmation at kitchen. Returning home.")
                    node.send_goal("home")  # Go home directly
                    continue  

                cancel_detected = False  # Flag for any cancelled orders

                for table in table_keys:
                    if node.send_goal(table):
                        confirmed = node.wait_for_confirmation(table)

                        if confirmed:
                            node.get_logger().info(f"Order confirmed at {table}. Proceeding to next.")
                        else:
                            node.get_logger().info(f"Order at {table} was cancelled!")
                            cancel_detected = True

                # If ANY table was cancelled, go to the kitchen before home
                if cancel_detected:
                    node.get_logger().info("One or more tables canceled. Returning to kitchen first.")
                    node.send_goal("kitchen")

                # Finally, go home
                node.send_goal("home")

    except KeyboardInterrupt:
        node.get_logger().info("Navigation stopped by user. Exiting...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
