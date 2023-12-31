import numpy
import sys
sys.path.insert(0,'../')
from DTOs.state_dto import state_dto

cos = numpy.cos
sin = numpy.sin
matix = numpy.array


class state_locations:
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
        self.__L1 = L1
        self.__L2 = L2
        self.__L3 = L3

    def calc(self, state_dto_obj : state_dto):
        '''
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
        beta = state_dto_obj.get_beta()
        theta1 = state_dto_obj.get_theta1()
        theta2 = state_dto_obj.get_theta2()
        theta3 = state_dto_obj.get_theta3()

        Ry = matix([[cos(beta), 0, sin(beta)],[0,1,0],[sin(beta), 0, cos(beta)]])

        alpha1 = theta1
        alpha2 = alpha1 + theta2 - (numpy.pi /2)
        alpha3 = alpha2 + theta3 - (numpy.pi /2)

        x1 = self.__L1 * cos(alpha1) + self.__L2 * cos(alpha2) + self.__L3 * cos(alpha3)
        x2 = self.__L1 * sin(alpha1) + self.__L2 * sin(alpha2) + self.__L3 * sin(alpha3)

        state = numpy.dot(Ry, matix([[x1],[x2],[0]]))

        state_dto_obj.set_Ma_p(state)
            

if __name__ == '__main__':
    x = state_locations()
    dto = state_dto(numpy.pi/4, numpy.pi/4, numpy.pi/4, numpy.pi)
    print(dto)
    print()
    print('Calculating: ')
    x.calc(dto)
    print(dto)
    