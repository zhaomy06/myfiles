import numpy as np


class Path:
    def __init__(self, file_name):
        self.file_name = file_name
        self.planning_x_list = []
        self.planning_y_list = []
        self.planning_h_list = []
        self.planning_v_list = []

    def path_preparation(self):
        data = np.loadtxt(self.file_name, delimiter=' ')
        self.planning_x_list = data[:, 0]
        self.planning_y_list = data[:, 1]
        self.planning_h_list = data[:, 2]
        self.planning_v_list = data[:, 3]
