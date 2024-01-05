'''
    This modules is the poblisher for the poss of the robot
'''
#python pkgs imports
import numpy
import threading
import time

#ros2 imports
import rclpy #pylint: disable=e0401
from rclpy.node import Node #pylint: disable=e0401
from geometry_msgs.msg import PoseStamped #pylint: disable=e0401
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup #pylint: disable=e0401
from rclpy.executors import MultiThreadedExecutor #pylint: disable=e0401

#Custom ros2 nodes imports
from state_sim_interface.srv import ArmyStateService #pylint: disable=e0401
from state_sim_interface.msg import PoseRequestPub #pylint: disable=e0401

#custom python imports
from state_space_algo_pub.DTOs.state_dto import state_dto

#create function pointers to make things esiar to code and read
cos = numpy.cos
sin = numpy.sin
matix = numpy.array

class state_locations(Node):
    '''
        This class is the Node that handles the state space simulation. It takes in the three thetas and beta then 
        returns the possition of the robot
    '''
    def __init__(self, L1 : float = 10, L2 : float = 10, L3 : float = 10) -> None:
        '''
            This class takes in a few theats and then out puts the final state of the mainipluator, and
            give the neede possition for checking obsitacal avoidancs. 

            Parameters:
                L1 : Length of first link -> float
                L2 : Length of sencond link -> float
                L3 : Length of third link -> float

            Inputs:
                theta1, theta2, theta3, beta
            Outputs:
                X1_p : fist link poss -> type numpy.array (x,y,z)
                X2_p : Second link poss -> type numpy.array (x,y,z)
                X3_p : Third link poss -> type numpy.array (x,y,z)
                Ma_p : Maniplator poss -> type numpy.array (x,y,z)
            
            NOTE : all the data is passed using the state_dto class
            NOTE: Everythin is done using Rads not degrees
        '''
        #set up paramiters for the class
        self.__L1 = L1
        self.__L2 = L2
        self.__L3 = L3

        #create the queue
        self.__queue = []
        self.__queue_lock = threading.Lock()
        
        #------ROS Set up section------
        #class the parent class constructor
        super().__init__(node_name="StateSpaceSim")

        #create call back groups
        self.__request_cb = MutuallyExclusiveCallbackGroup()
        self.__processing_cb = MutuallyExclusiveCallbackGroup()

        #create publisher, this will the position to the rest of the system.
        self.pub_pos = self.create_publisher(PoseRequestPub, "pose_request", 10)

        #create service for reciving the possition calculations
        self.srv_pos = self.create_service(ArmyStateService, "army_state_service", self.collect_request, callback_group = self.__request_cb)

        #create a call back timer for processing the services
        self.processing_timer = self.create_timer(timer_period_sec=1, callback=self.process, callback_group=self.__processing_cb)
    def collect_request(self, request, response):
        '''
            This function collects the request and populates it into the queue for futer processing
        '''
        
        beta = request.beta
        theta1 = request.theta1
        theta2 = request.theta2
        theta3 = request.theta3
        req_id = request.id
        with self.__queue_lock:
            #collect the request
            #append the request to the queue
            self.__queue.append([beta, theta1, theta2, theta3, req_id])
        #tell the client the request was recived
        recived = True
        response.recived = recived
        return response
    def process(self):
        '''
            This function handles the requsts that have been passed into the queue. 
            It then calls the calc function 
        '''
        length = 0
        with self.__queue_lock:
            length = len(self.__queue)
        for _ in range(length):
            with self.__queue_lock:
                temp = self.__queue.pop(0)
            self.calc(temp)
    def calc(self, request):
        '''
            ROS2 NOTE: This function is what is tied to the service call back

            This function uses the state_dto give to it and then populats the 
            final state informations.

            Input :
                state_dto_obj : not populated, but has inital condtions -> state_dto
            Output :
                state_dto_obj : populated with final possitions -> state_dto

            Equations:

                alpha1 = theta1
                alpha_n = alpha_n-1 + theta_n - (pi/2)

                Ry = [cos(beta), 0, sin(beta)]
                     [   0,      1,    0     ]
                     [sin(beta), 0, cos(beta)]

                [x]   [L1*cos(alpha1)+L2*cos(alpha2)+L3*cos(alpha3)]
                [y] = [L1*sin(alpha1)+L2*sin(alpha2)+L3*sin(alpha3)] * [Ry]
                [z]   [                 0                          ]
            
        '''

        #get our rotation matix
        beta = request[0]
        theta1 = request[1]
        theta2 = request[2]
        theta3 = request[3]
        req_id = request[4]

        #create the state obj to store the data
        Ry = matix([[cos(beta), 0, sin(beta)],
                    [0,1,0],
                    [sin(beta), 0, cos(beta)]])

        alpha1 = theta1
        alpha2 = alpha1 + theta2 - (numpy.pi /2)
        alpha3 = alpha2 + theta3 - (numpy.pi /2)


        #create X1 pose stamp
        x1 = self.__L1 * cos(alpha1)
        x2 = self.__L1 * sin(alpha1)

        state = numpy.dot(Ry, matix([[x1],[x2],[0]]))

        pose_x1 = PoseStamped()
        pose_x1.pose.orientation.x = state[0][0]
        pose_x1.pose.orientation.y = state[1][0]
        pose_x1.pose.orientation.z = state[2][0]

        #create X2 pose stamp
        x1 = x1 + self.__L2 * cos(alpha2)
        x2 = x2 + self.__L2 * sin(alpha2)

        state = numpy.dot(Ry, matix([[x1],[x2],[0]]))

        pose_x2 = PoseStamped()
        pose_x2.pose.orientation.x = state[0][0]
        pose_x2.pose.orientation.y = state[1][0]
        pose_x2.pose.orientation.z = state[2][0]

        #create X3 pose stamp
        x1 = x1 + self.__L3 * cos(alpha3)
        x2 = x2 + self.__L3 * sin(alpha3)

        state = numpy.dot(Ry, matix([[x1],[x2],[0]]))

        pose_x3 = PoseStamped()
        pose_x3.pose.orientation.x = state[0][0]
        pose_x3.pose.orientation.y = state[1][0]
        pose_x3.pose.orientation.z = state[2][0]

        #create Ma pose stamp
        pose = PoseStamped()
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0

        #create the message
        pos_request_mesage = PoseRequestPub()
        pos_request_mesage.theta1 = theta1
        pos_request_mesage.theta2 = theta2
        pos_request_mesage.theta3 = theta3
        pos_request_mesage.beta = beta
        pos_request_mesage.id = req_id
        pos_request_mesage.pose_stamped = pose #assign poss of manipulator
        pos_request_mesage.pose_stamped_l1 = pose_x1 #assign poss of link 1
        pos_request_mesage.pose_stamped_l2 = pose_x2 #assign poss of link 2
        pos_request_mesage.pose_stamped_l3 = pose_x3 #assign poss of link 3
        pos_request_mesage.vaild = True

        #publish message
        self.pub_pos.publish(pos_request_mesage)        
def main(args=None):
    '''
        main function, starts the node
    '''
    rclpy.init(args=args)
    state_service = state_locations()

    # rclpy.spin(state_service)

    # rclpy.shutdown()
    ros_exec = MultiThreadedExecutor()
    ros_exec.add_node(state_service)
    ros_thread = threading.Thread(target=ros_exec.spin, daemon=True)
    ros_thread.start()
    while rclpy.ok():
        time.sleep(1)
    ros_thread.join()
if __name__ == '__main__':
    main()
    