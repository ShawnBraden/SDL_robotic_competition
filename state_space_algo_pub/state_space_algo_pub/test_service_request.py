'''
    test function for the service request node

    Calls the services, then prints what is returned.
'''
#ros imports
import rclpy #pylint: disable=e0401
from rclpy.node import Node #pylint: disable=e0401

#other ros2 nodes imports
from state_sim_interface.srv import ArmyStateService #pylint: disable=e0401
from state_sim_interface.msg import PoseRequestPub #pylint: disable=e0401

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

        #create a subscriber to collect the pos_request publisher
        self.subcription =  self.create_subscription(
            PoseRequestPub,
            'pose_request',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        '''
            This function is the callback for when a pos_request message is published
        '''
        print(f'Pos recived: \n{msg}')

    def send_request(self, theta1, theta2, theta3, beta, req_id):
        '''
            Sends a request then waits for the response. 
        '''
        self.req.theta1 = theta1
        self.req.theta2 = theta2
        self.req.theta3 = theta3
        self.req.beta = beta
        self.req.id = req_id
        
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
    for i in range(10000):
        response = client.send_request(float(sys.argv[1]) + i * numpy.pi / 180,float(sys.argv[2]) + i * numpy.pi / 180,float(sys.argv[3]) + i * numpy.pi / 180,float(sys.argv[4]) + i * numpy.pi / 180, i)
    client.get_logger().info(f'Possition:\n {response}')

    #spin node and wait for response
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
