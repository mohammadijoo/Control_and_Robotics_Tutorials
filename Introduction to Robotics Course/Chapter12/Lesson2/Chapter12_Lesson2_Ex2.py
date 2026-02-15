from rclpy.action import ActionServer, ActionClient
from example_interfaces.action import Fibonacci

class FibActionServer(Node):
    def __init__(self):
        super().__init__('fib_server')
        self.server = ActionServer(
            self, Fibonacci, '/fib', self.execute_cb)

    async def execute_cb(self, goal_handle):
        order = goal_handle.request.order
        seq = [0, 1]
        for i in range(2, order):
            seq.append(seq[-1] + seq[-2])
            goal_handle.publish_feedback(Fibonacci.Feedback(sequence=seq))
            await rclpy.sleep(0.1)
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)

class FibActionClient(Node):
    def __init__(self):
        super().__init__('fib_client')
        self.client = ActionClient(self, Fibonacci, '/fib')

    async def send_goal(self, n):
        await self.client.wait_for_server()
        goal = Fibonacci.Goal(order=n)
        gh_future = self.client.send_goal_async(goal, feedback_callback=self.fb_cb)
        goal_handle = await gh_future
        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result.sequence

    def fb_cb(self, feedback_msg):
        self.get_logger().info(f"feedback={feedback_msg.feedback.sequence}")
      
