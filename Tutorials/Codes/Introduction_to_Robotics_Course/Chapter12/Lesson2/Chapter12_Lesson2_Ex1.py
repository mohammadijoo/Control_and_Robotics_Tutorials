from example_interfaces.srv import AddTwoInts

class AdderServer(Node):
    def __init__(self):
        super().__init__('adder_server')
        self.srv = self.create_service(AddTwoInts, '/add', self.handle)

    def handle(self, req, res):
        res.sum = req.a + req.b
        return res

class AdderClient(Node):
    def __init__(self):
        super().__init__('adder_client')
        self.cli = self.create_client(AddTwoInts, '/add')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /add ...")

    def call_add(self, a, b):
        req = AddTwoInts.Request()
        req.a, req.b = a, b
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().sum
      
