'''
    test function for the service request node

    Calls the services, then prints what is returned.
'''
#ros imports
import rclpy #pylint: disable=e0401
from rclpy.node import Node #pylint: disable=e0401

#other ros2 nodes imports
from state_sim_interface.srv import CreateRRTService #pylint: disable=e0401

#python imports
import numpy
import sys

class test_service_request(Node):
    '''
        This Node is used to test the the rrt_tree Node.
        It just sends requests to the nodes. 
    '''
    def __init__(self):
        '''
            This class subscribes to the server service
        '''
        super().__init__('state_location_service_test_client')
        self.cli = self.create_client(CreateRRTService, 'create_rrt')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CreateRRTService.Request()
        self.future = None # define for future use

    def send_request(self, seed):
        '''
            Sends a request then waits for the response. 
        '''
        self.req.seed = seed
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    '''
        This function just fires off a bunch of requests and then collects the responses to verifiy that they are correct. 
    '''
    rclpy.init(args=args)
    client = test_service_request()
    
    #send request
    response = client.send_request(int (sys.argv[1]))

    #spin node and wait for response
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
