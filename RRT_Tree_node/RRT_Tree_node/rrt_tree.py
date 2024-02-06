'''
    This Node creates the tree sturcture for the Robotic arm.

    NOTE: This class knows about the state space and then calculates where 
    it is possible paths to move accross. 
'''
#ros imports
import rclpy #pylint: disable=e0401
import random
import numpy as np
from rclpy.node import Node #pylint: disable=e0401
from geometry_msgs.msg import PoseStamped #pylint: disable=e0401
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup #pylint: disable=e0401
from rclpy.executors import MultiThreadedExecutor #pylint: disable=e0401

#other ros2 nodes imports
from state_sim_interface.srv import ArmyStateService #pylint: disable=e0401
from state_sim_interface.srv import CreateRRTService #pylint: disable=e0401
from state_sim_interface.msg import PoseRequestPub #pylint: disable=e0401

#custom python imports
from state_space_algo_pub.DTOs.state_dto import state_dto #pylint: disable=e0401

#python imports
import threading
import time

#create function pointers to make things esiar to code and read
cos = np.cos
sin = np.sin
matix = np.array

class rrt_tree(Node):
    '''
        This class builds and maintains a tree structure,
        You can ask it for a path between two poins, it will then calculate 
        it and return the path.
    '''
    def __init__(self, itterations : int = 100000, seed : int = 10, max_delta : float = np.pi/90, L1 : float = 10, L2 : float = 10, L3 : float = 10,  min_theta1 : float = 0, min_theta2 : float = 0, min_theta3 : float = 0, min_beta : float = 0, max_theta1 : float = np.pi/2, max_theta2 : float = np.pi/2, max_theta3 : float = np
                 .pi/2, max_beta : float = 2*np.pi) -> None:
        '''
            Parameters:
                itterations : Number of itterations the tree runs for -> int
                L1 : Length of first link -> float
                L2 : Length of sencond link -> float
                L3 : Length of third link -> float
                max_delta : maximum amount any angle can change between states ->
                min_theta1 : maximum relative angle of the first joint -> float
                min_theta2 : maximum relative angle of the second joint -> float
                min_theta3 : maximum relative angle of the third joint -> float
                min_beta : maximum rotational angle of the base (should allow full rotation) -> float
                max_theta1 : maximum relative angle of the first joint -> float
                max_theta2 : maximum relative angle of the second joint -> float
                max_theta3 : maximum relative angle of the third joint -> float
                max_beta : maximum rotational angle of the base (should allow full rotation) -> float
                Seed : Random seed -> int
            Inputs: 
                None
            Output:
                Path between two paths
        '''
        #set up paramiters for the class
        self.__itterations = itterations

        self.__max_delta = max_delta

        self.__L1 = L1
        self.__L2 = L2
        self.__L3 = L3

        self.__min_theta1 = min_theta1
        self.__min_theta2 = min_theta2
        self.__min_theta3 = min_theta3
        self.__min_beta = min_beta

        self.__max_theta1 = max_theta1
        self.__max_theta2 = max_theta2
        self.__max_theta3 = max_theta3
        self.__max_beta = max_beta

        self.__seed = seed
        self.__current_id = 0
        self.__tree_data = {}

        self.__generated = False



        #create the queues
        self.__random_points_queue = []
        self.__random_points_lock = threading.Lock()

        #-------Ros Set up settion-------
        super().__init__('state_location_service_test_client')

        #set up service for requesting the state calculations
        # self.cli = self.create_client(ArmyStateService, 'army_state_service')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = ArmyStateService.Request()
        # self.future = None # define for future use

        #create a subscriber to collect the pos_request publisher
        # self.subcription =  self.create_subscription(PoseRequestPub, 'pose_request', self.listener_callback, 10)

        #set up call back group to handle the RRT creations
        self.__rrt_cb = MutuallyExclusiveCallbackGroup()

        #create a service for calling rrt creation 
        self.srv_pos = self.create_service(CreateRRTService, "create_rrt", self.create_rrt, callback_group = self.__rrt_cb)     
    def listener_callback(self, msg):
        '''
            This function recives a 'pose_request' when it is published and then saves it into the queue
        '''
        # Create dto before passing it onto the queue
        dto = state_dto(msg.theta1, msg.theta2, msg.theta3, msg.beta)
        dto.set_id(msg.id)

        # get the Ma possition
        x = msg.pose_stamped.orientation.x
        y = msg.pose_stamped.orientation.y
        z= msg.pose_stamped.orientation.z
        dto.set_Ma_p(matix([x, y, z]))

        #get X1 possition
        x = msg.pose_stamped_l1.orientation.x
        y = msg.pose_stamped_l1.orientation.y
        z= msg.pose_stamped_l1.orientation.z
        dto.set_X1_p(matix([x, y, z]))

        #get X2 possition
        x = msg.pose_stamped_l2.orientation.x
        y = msg.pose_stamped_l2.orientation.y
        z= msg.pose_stamped_l2.orientation.z
        dto.set_X2_p(matix([x, y, z]))

        #get X3 possition
        x = msg.pose_stamped_l3.orientation.x
        y = msg.pose_stamped_l3.orientation.y
        z= msg.pose_stamped_l3.orientation.z
        dto.set_X3_p(matix([x, y, z]))

        #get the vaild out of the msg
        dto.set_vaild(msg.vaild)

        #add the dto to the queue
        with self.__random_points_lock:
            self.__random_points_queue.append(dto)
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
    def generate_random_angles(self):
        '''
            this class generates the random angles for the seed
            inputs:
                none
            outputs:
                random angles theta1, theta2, theta3, beta
        '''
        theta1 = (random.random() * (self.__max_theta1 - self.__min_theta1)) + self.__min_theta1
        theta2 = (random.random() * (self.__max_theta2 - self.__min_theta2)) + self.__min_theta2
        theta3 = (random.random() * (self.__max_theta3 - self.__min_theta3)) + self.__min_theta3
        beta = (random.random() * (self.__max_beta - self.__min_beta)) + self.__min_beta

        return theta1, theta2, theta3, beta
    def create_rrt(self, request, response):
        seed = request.seed
        random.seed(seed)
        i_theta1, i_theta2, i_theta3, i_beta = self.generate_random_angles()
        self.__tree_data["root"] = state_dto(i_theta1, i_theta2, i_theta3, i_beta, "root", " ")
        print('Called rrt')

        for i in range(self.__itterations):

            current_id = str(self.__current_id)
            c_theta1, c_theta2, c_theta3, c_beta = self.generate_random_angles()
            minimum_angle_difference = float('inf')
            minimum_angle_parent_id = ""

            goal_theta1, goal_theta2, goal_theta3, goal_beta = c_theta1, c_theta2, c_theta3, c_beta

            parent_id = "root"

            for potential_parent_id in self.__tree_data:
                potential_parent = self.__tree_data[potential_parent_id]
                new_theta1 = potential_parent.get_theta1()
                new_theta2 = potential_parent.get_theta2()
                new_theta3 = potential_parent.get_theta3()
                new_beta = potential_parent.get_beta()
                angle_difference = np.sqrt(((new_theta1 - c_theta1)**2)+((new_theta2 - c_theta2)**2)+((new_theta3 - c_theta3)**2)+((new_beta - c_beta)**2))
                if angle_difference < minimum_angle_difference:
                    minimum_angle_difference = angle_difference

                    minimum_angle_parent_id = potential_parent_id

                    goal_theta1, goal_theta2, goal_theta3, goal_beta = new_theta1, new_theta2, new_theta3, new_beta

            delta_theta1 = goal_theta1 - c_theta1
            delta_theta2 = goal_theta2 - c_theta2
            delta_theta3 = goal_theta3 - c_theta3
            delta_beta = goal_beta - c_beta
                # correct changes in angles that are greater than the maximum
            if abs(delta_theta1) > self.__max_delta:
                delta_theta1 = (delta_theta1/abs(delta_theta1))*self.__max_delta

            if abs(delta_theta2) > self.__max_delta:
                delta_theta2 = (delta_theta2/abs(delta_theta2))*self.__max_delta

            if abs(delta_theta3) > self.__max_delta:
                delta_theta3 = (delta_theta3/abs(delta_theta3))*self.__max_delta

            if abs(delta_beta) > self.__max_delta:
                delta_beta = (delta_beta/abs(delta_beta))*self.__max_delta

            next_theta1 = c_theta1 + delta_theta1
            next_theta2 = c_theta2 + delta_theta2
            next_theta3 = c_theta3 + delta_theta3
            next_beta = c_beta + delta_beta

            # make sure that the new angles don't exceed the limits
            if (next_theta1 < self.__min_theta1):
                next_theta1 = self.__min_theta1
            elif (next_theta1 > self.__max_theta1):
                next_theta1 = self.__max_theta1

            if (next_theta2 < self.__min_theta2):
                next_theta2 = self.__min_theta2
            elif (next_theta2 > self.__max_theta2):
                next_theta2 = self.__max_theta2

            if (next_theta3 < self.__min_theta3):
                next_theta3 = self.__min_theta3
            elif (next_theta3 > self.__max_theta3):
                next_theta3 = self.__max_theta3

            if (next_beta < self.__min_beta):
                next_beta = self.__min_beta
            elif (next_beta > self.__max_beta):
                next_beta = self.__max_beta

            #print(f"key = {minimum_angle_parent_id}")
            if len(self.__tree_data) != 0:
                self.__tree_data[current_id] = state_dto(next_theta1, next_theta2, next_theta3, next_beta, current_id, minimum_angle_parent_id)
            
                self.__tree_data[minimum_angle_parent_id].add_child_id(current_id)
            
            self.__current_id += 1

        print(self.__tree_data["root"].get_children_id())
        print(self.__tree_data["0"].get_children_id())
        print(self.__tree_data["1"].get_children_id())
        print(self.__tree_data["2"].get_children_id())
        print(self.__tree_data["3"].get_children_id())


        self.__generated = True
        response.success = True
        return response
    
    def rewire(self, max_conceptual_distance):
        if self.__generated:


            # this radius is twice the theoretical maximum conseptual distance from the previous state
            radius_of_rewire = 4*self.__max_delta
            

            for current_state_id in self.__tree_data:
                current_state = self.__tree_data[current_state_id]
                current_theta1 = current_state.get_theta1()
                current_theta2 = current_state.get_theta2()
                current_theta3 = current_state.get_theta3()
                current_beta = current_state.get_beta()

                for next_state_id in self.__tree_data:
                    next_state = self.__tree_data[next_state_id]
                    conceptual_distance = np.sqrt(((next_state.get_theta1()-current_state.get_theta1())**2)+
                                                  ((next_state.get_theta2()-current_state.get_theta2())**2)+
                                                  ((next_state.get_theta3()-current_state.get_theta3())**2)+
                                                  ((next_state.get_beta()-current_state.get_beta())**2))
                    if (conceptual_distance <= max_conceptual_distance):
                        current_state.add_child_id(next_state_id)
                        next_state.add_parent_id(current_state_id)

                        print("here")


                        





def main(args=None):
    '''
        main function, starts the node
    '''
    rclpy.init(args=args)
    state_service = rrt_tree()
    ros_exec = MultiThreadedExecutor()
    ros_exec.add_node(state_service)
    ros_thread = threading.Thread(target=ros_exec.spin, daemon=True)
    ros_thread.start()
    while rclpy.ok():
        time.sleep(1)
    ros_thread.join()
if __name__ == '__main__':
    main()
