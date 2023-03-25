'''
@Author: Xiaoxin Mi
@Email: mixiaoxin@whu.edu.cn; mixiaoxin_2012@163.com
@Brief: Read and write .PosT files
'''

import numpy as np

class IO_PosT():
    def __int__(self):
        self.hearders = []
        self.keys = []
        self.poses = []

    def read_PosT(self, filename):
        # load data into lists, which are seperated by the "\n"
        my_file = open(filename, "r")
        data = my_file.read()
        data_into_list = data.split("\n")
        my_file.close()

        self.hearders = data_into_list[0:2] # The first 21 lines are headers
        self.keys = data_into_list[0]
        self.poses = data_into_list[2:]

        # extract values in each pose
        self.poses = np.genfromtxt(filename, skip_header=2)
        # for idx in range(len(self.poses)):
        #     pos = self.poses[idx]
        #     self.poses[idx] = [item for item in pos.split('')]
        return self.poses

    def write_PosT(self, filename, post_list=None):
        # write .PosT file, including headers and poses
        with open(filename, "w") as f:
            # write headers
            for h in self.hearders:
                f.write(h)
            # write poses
            pose_to = []
            if post_list == None:
                pose_to = self.poses
            else:
                pose_to = post_list
            for pos in pose_to:
                str_out = ''.join(str(item) for item in pos)
                str_out += '\n'
                f.write(str_out)
        f.close()


    def write_XYZT(self, filename, post_list=None):
        # write X, Y, Z, T into file
        with open(filename, "w") as f:
            pose_to = []
            if post_list == None:
                pose_to = self.poses
            else:
                pose_to = post_list
            # write xyzt in poses
            for pos in pose_to:
                str_out = str() + ", " + str() + ", "+ str() + ", " + str()
                str_out += '\n'
                f.write(str_out)
        f.close()

    def get_poses(self):
        return self.poses