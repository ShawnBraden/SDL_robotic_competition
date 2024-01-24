'''
    This class is used for passing information on a single state of the robot arm
'''
import numpy

class state_dto:
    '''
        This class holds all the info for the state of the robot arm 
    '''
    def __init__(self, theta1:float, theta2:float, theta3:float, beta:float, my_id:str, parent_id:str) -> None:
        '''
            This class holds all the need information for the state.

            Inputs:
                theta1, theta2, theta3, beta, my_id, parent_id
            Outputs:
                X1_p : fist link poss -> type tuple (x,y,z)
                X2_p : Second link poss -> type tuple (x,y,z)
                X3_p : Third link poss -> type tuple (x,y,z)
                Ma_p : Maniplator poss -> type tuple (x,y,z)
        '''
        self.__id = my_id
        self.__parents_id = [parent_id]
        self.__children_id = []
        self.__theta1 = theta1
        self.__theta2 = theta2
        self.__theta3 = theta3
        self.__beta = beta
        self.__X1_p = -1
        self.__X2_p = -1
        self.__X3_p = -1
        self.__Ma_p = -1
        self.__vaild = False
        
    ### GETTERS ###
    def get_theta1(self):
        # pylint: disable=missing-function-docstring
        return self.__theta1
    def get_theta2(self):
        # pylint: disable=missing-function-docstring
        return self.__theta2
    def get_theta3(self):
        # pylint: disable=missing-function-docstring
        return self.__theta3
    def get_beta(self):
        # pylint: disable=missing-function-docstring
        return self.__beta
    def get_X1_p(self):
        # pylint: disable=missing-function-docstring
        return self.__X1_p
    def get_X2_p(self):
        # pylint: disable=missing-function-docstring
        return self.__X2_p
    def get_X3_p(self):
        # pylint: disable=missing-function-docstring
        return self.__X3_p
    def get_Ma_p(self):
        # pylint: disable=missing-function-docstring
        return self.__Ma_p
    def is_vaild(self):
        # pylint: disable=missing-function-docstring
        return self.__vaild
    def get_id(self):
        # pylint: disable=missing-function-docstring
        return self.__id
    ### Setters ###
    def set_theta1(self, theta1:float):
        # pylint: disable=missing-function-docstring
        self.__theta1 = theta1
    def set_theta2(self, theta2:float):
        # pylint: disable=missing-function-docstring
        self.__theta2 = theta2
    def set_theta3(self, theta3:float):
        # pylint: disable=missing-function-docstring
        self.__theta3 = theta3
    def set_beta(self, beta:float):
        # pylint: disable=missing-function-docstring
        self.__beta = beta
    def set_X1_p(self, X1_p:numpy.array):
        # pylint: disable=missing-function-docstring
        self.__X1_p = X1_p
    def set_X2_p(self, X2_p:numpy.array):
        # pylint: disable=missing-function-docstring
        self.__X2_p = X2_p
    def set_X3_p(self, X3_p:numpy.array):
        # pylint: disable=missing-function-docstring
        self.__X3_p = X3_p
    def set_Ma_p(self, Ma_p:numpy.array):
        # pylint: disable=missing-function-docstring
        self.__Ma_p = Ma_p
    def set_vaild(self, vaild:bool):
        # pylint: disable=missing-function-docstring
        self.__vaild = vaild
    def set_id(self, my_id:str):
        # pylint: disable=missing-function-docstring
        self.__id = my_id
    def get_my_id(self):
        # pylint: disable=missing-function-docstring
        return self.__id
    def get_og_parent_id(self):
        # pylint: disable=missing-function-docstring
        return self.__parents_id[0]
    
    def get_parents_id(self):
        # pylint: disable=missing-function-docstring
        return self.__parents_id

    def set_og_parent_id(self, parent_id:str):
        # pylint: disable=missing-function-docstring
        self.__parents_id[0] = parent_id
    def add_parent_id(self, parent_id:str):
        if self.__children_id.count(parent_id) == 0 and self.__parents_id.count(parent_id) == 0 and parent_id != self.__id and parent_id != "root":
            self.__parents_id.append(parent_id)


    def add_child_id(self, child_id:str):
        # pylint: disable=missing-function-docstring
        if self.__children_id.count(child_id) == 0 and self.__parents_id.count(child_id) == 0 and child_id != self.__id and child_id != "root":
            self.__children_id.append(child_id)
    def remove_child_id(self, child_id:str):
        # pylint: disable=missing-function-docstring
        if self.__children_id.count(child_id) != 0:
            self.__children_id.remove(child_id)
    def get_children_id(self):
        # pylint: disable=missing-function-docstring
        return self.__children_id
    def __str__(self) -> str:
        '''
            This function prints the dto 
        '''
        return f'State_DTO Report:\n\tTheta 1: {self.__theta1}\n\tTheta 2: {self.__theta2}\n\tTheta 3: {self.__theta3}\n\tBeta: {self.__beta}\n\tMa_poss: {self.__Ma_p}\n\tX1_poss: {self.__X1_p}\n\tX2_poss: {self.__X2_p}\n\tX3_poss: {self.__X3_p}\n\tIs_Vaild: {self.__vaild}'
    