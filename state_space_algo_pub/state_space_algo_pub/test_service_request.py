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
import sys

class test_service_request(Node):
    '''
        This Node is used to test the the state_location Node.
        It just sends requests then checks there values. 
    '''
    def __init__(self):
        '''
            This class subscribes to the server service
        '''
        super().__init__('state_location_service_test_client')
        self.cli = self.create_client(ArmyStateService, 'army_state_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ArmyStateService.Request()
        self.future = None # define for future use

    def send_request(self, theta1, theta2, theta3, beta):
        '''
            Sends a request then waits for the response. 
        '''
        self.req.theta1 = theta1
        self.req.theta2 = theta2
        self.req.theta3 = theta3
        self.req.beta = beta

        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    '''
        This function just fires off a bunch of requests and then collects the responses to verifiy that they are correct. 
    '''
    rclpy.init(args=args)

    client = test_service_request()
    for _ in range(100):
        response = client.send_request(float(sys.argv[1]) * numpy.pi / 180,float(sys.argv[2])* numpy.pi / 180,float(sys.argv[3])* numpy.pi / 180,float(sys.argv[4])* numpy.pi / 180)
    client.get_logger().info(f'Possition:\n {response}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
