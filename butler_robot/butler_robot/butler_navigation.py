import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.client.wait_for_server()

        self.locations = {
            "kitchen": (4.28, -0.67, 0.7071, 0.7071),
            "table1": (3.40, 3.48, 0.7071, 0.7071),
            "table2": (-3.11, 3.38, -0.99, -0.03),
            "table3": (-2.88, -0.79, -0.99, 0.12),
            "home": (0.0, 0.0, 0.0, 1.57)
        }

    def send_goal(self, location):
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
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f"Sending goal to {location}...")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Reached {location} successfully.")
            return True
        else:
            self.get_logger().error(f"Failed to reach {location}. Status: {result.status}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()

    try:
        # Example: Send to kitchen once when started
        node.send_goal('kitchen')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
