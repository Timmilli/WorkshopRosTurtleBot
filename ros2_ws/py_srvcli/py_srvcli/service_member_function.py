from service_interfaces.srv import BoolResponse

import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(BoolResponse, 'bool_response', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request\n%s %d' % (request.computer_name, request.domain_id))
        if request.computer_name == "circuit" and request.domain_id == 2:
            response.result = True
        else:
            response.result = False

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
