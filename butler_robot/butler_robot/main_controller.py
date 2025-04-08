import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from butler_bot_interfaces.action import NavigateTables
from butler_bot_interfaces.srv import ConfirmArrival
from butler_robot.butler_navigation import NavGoalSender

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self._action_server = ActionServer(
            self,
            NavigateTables,
            'navigate_tables',
            self.execute_callback
        )
        self.nav_sender = NavGoalSender()
        self.cli = self.create_client(ConfirmArrival, 'confirm_arrival')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for confirmation service...')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received table navigation request.")
        tables = goal_handle.request.tables
        cancel_detected = False

        if not self.nav_sender.send_goal('kitchen'):
            goal_handle.abort()
            return NavigateTables.Result(success=False)

        if not await self.wait_for_confirmation('kitchen'):
            self.nav_sender.send_goal('home')
            goal_handle.abort()
            return NavigateTables.Result(success=False)

        for table in tables:
            goal_handle.publish_feedback(NavigateTables.Feedback(current_table=table))
            if self.nav_sender.send_goal(table):
                confirmed = await self.wait_for_confirmation(table)
                if not confirmed:
                    cancel_detected = True

        if cancel_detected:
            self.nav_sender.send_goal('kitchen')

        self.nav_sender.send_goal('home')
        goal_handle.succeed()

        return NavigateTables.Result(success=True)

    async def wait_for_confirmation(self, location):
        req = ConfirmArrival.Request()
        req.location = location
        future = self.cli.call_async(req)
        await future
        return future.result().confirmed

def main(args=None):
    rclpy.init(args=args)
    node = MainController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
