'''
    test function for the service request node

    Calls the services, then prints what is returned.
'''
#ros imports
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#other ros2 nodes imports
from state_sim_interface.action import ArmyStateAction

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
        self.cli = ActionClient(self, ArmyStateAction, 'army_state_action')
        self.__num_handled = 0

    def send_request(self, theta1, theta2, theta3, beta):
        '''
            Sends a request then waits for the response. 
        '''
        goal = ArmyStateAction.Goal()
        goal.theta1 = theta1
        goal.theta2 = theta2
        goal.theta3 = theta3
        goal.beta = beta

        self.cli.wait_for_server()
        
        self._send_goal_future = self.cli.send_goal_async(goal)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.pose_stamped))
        self.__num_handled += 1
        print(f'Number Handled: {self.__num_handled}')

def main(args=None):
    '''
        This function just fires off a bunch of requests and then collects the responses to verifiy that they are correct. 
    '''
    rclpy.init(args=args)

    client = test_service_request()
    for i in range(100):
        future = client.send_request(float(sys.argv[1]) + i * numpy.pi / 180,float(sys.argv[2]) + i * numpy.pi / 180,float(sys.argv[3]) + i * numpy.pi / 180,float(sys.argv[4]) + i * numpy.pi / 180)
        client.get_logger().info(f'Possition {i}:\n {future}')

    rclpy.spin(client)

if __name__ == '__main__':
    main()
