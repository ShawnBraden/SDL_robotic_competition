'''
    test function for the service request node

    Calls the services, then prints what is returned.
'''
#ros imports
import rclpy
from rclpy.node import Node

#other ros2 nodes imports
from state_sim_interface.srv import ArmyStateService

#python imports
import numpy

class test_service_request(Node):
    def __init__(self):
        super().__init__('state_location_service_test_client')
        self.cli = self.create_client(ArmyStateService, 'army_state_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ArmyStateService.Request()

    def send_request(self):
        self.req.theta1 = numpy.pi / 4
        self.req.theta2 = numpy.pi / 4
        self.req.theta3 = numpy.pi / 4
        self.req.beta = numpy.pi

        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    client = test_service_request()
    response = client.send_request()
    client.get_logger().info(f'Possition:\n {response}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()