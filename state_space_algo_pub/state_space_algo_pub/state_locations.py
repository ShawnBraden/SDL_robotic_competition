'''
    This modules is the poblisher for the poss of the robot
'''
#python pkgs imports
import numpy

#ros2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped #pylint: disable=e0401

#other ros2 nodes imports
from state_sim_interface.srv import ArmyStateService

#custom python imports
from state_space_algo_pub.DTOs.state_dto import state_dto

#create function pointers to make things esiar to code and read
cos = numpy.cos
sin = numpy.sin
matix = numpy.array


class state_locations(Node):
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
            NOTE: Everythin is done using Rads
        '''
        #set up paramiters for the class
        self.__L1 = L1
        self.__L2 = L2
        self.__L3 = L3
        
        #------ROS Set up section------
        #class the parent class constructor
        super().__init__(node_name="StateSpaceSim")

        #create publisher, this will publish the position to the rest of the system.
        self.srv_pos = self.create_service(ArmyStateService, "army_state_service", self.calc)

    def calc(self, request, response):
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
        beta = request.beta
        theta1 = request.theta1
        theta2 = request.theta2
        theta3 = request.theta3

        #create the state obj to store the data
        state_dto_obj = state_dto(theta1=theta1, theta2=theta2, theta3=theta3, beta=beta)

        Ry = matix([[cos(beta), 0, sin(beta)],[0,1,0],[sin(beta), 0, cos(beta)]])

        alpha1 = theta1
        alpha2 = alpha1 + theta2 - (numpy.pi /2)
        alpha3 = alpha2 + theta3 - (numpy.pi /2)

        x1 = self.__L1 * cos(alpha1) + self.__L2 * cos(alpha2) + self.__L3 * cos(alpha3)
        x2 = self.__L1 * sin(alpha1) + self.__L2 * sin(alpha2) + self.__L3 * sin(alpha3)

        state = numpy.dot(Ry, matix([[x1],[x2],[0]]))

        state_dto_obj.set_Ma_p(state)

        pose = PoseStamped()
        pose.pose.orientation.x = state[0][0]
        pose.pose.orientation.y = state[1][0]
        pose.pose.orientation.z = state[2][0]

        response.pose_stamped = pose #assign return val to the response

        return response

def main(args=None):
    '''
        main function, starts the node, right now it is just single threaded 
    '''
    rclpy.init(args=args)
    state_service = state_locations()

    rclpy.spin(state_service)

    rclpy.shutdown()

if __name__ == '__main__':
    x = state_locations()
    dto = state_dto(numpy.pi/4, numpy.pi/4, numpy.pi/4, numpy.pi)
    print(dto)
    print()
    print('Calculating: ')
    x.calc(dto)
    print(dto)
    