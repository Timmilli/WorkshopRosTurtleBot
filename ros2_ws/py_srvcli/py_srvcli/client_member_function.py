import sys

from service_interfaces.srv import BoolResponse
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(BoolResponse, 'bool_response')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = BoolResponse.Request()

    def send_request(self, a, b):
        self.req.computer_name = a
        self.req.domain_id = b
        return self.cli.call_async(self.req)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(str(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of bool_response: %s %d = %r' % (str(sys.argv[1]), int(sys.argv[2]), response.result))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
