import numpy as np
import matplotlib.pyplot as plt

class global_map_obj():
    def __init__(self, length:int = 2, resolution:float= 0.01):
        # Define array dimensions
        self.__resolution = resolution
        self.__length = length
        self.__max_x = int(length / resolution)
        print(type(self.__max_x))
        self.__max_y = int(length / resolution)
        self.__max_z = int(length / resolution)

        # Create 3D NumPy array filled with zeros
        self.__map_obj = np.zeros((self.__max_x, self.__max_y, self.__max_z))
    def set_pos(self, x, y, z):
        if not (x == 0 and y == 0 and z == 0):
            x = int((x / self.__resolution) + (self.__max_x)/ 2) - 1
            y = int(y / self.__resolution) - 1
            z = int((z/ self.__resolution) + (self.__max_z) / 2) -1 
            if x < self.__max_x and y < self.__max_y and z < self.__max_z and x >= 0 and y >= 0 and z >= 0:
                # print(f'{x},{y},{z}')
                self.__map_obj[x, y, z] = 1
    def display_graph(self):
        # print(self.__map_obj)
        # Create figure and axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Iterate over the array and plot points where value is 1
        for x in range(self.__max_x):
            for y in range(self.__max_y):
                for z in range(self.__max_z):
                    if self.__map_obj[x, y, z] == 1:
                        ax.scatter(x, y, z, color='blue')  # Plotting point
        # Set labels and show plot
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title('3D Scatter Plot')
        plt.show()
