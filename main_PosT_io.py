'''
@ Author: Xiaoxin Mi
'''
import os
import numpy as np
from data import CoorData
from IO_PosT import IO_PosT



'''
@ Brief: This function aims for loading and saving PosT.
'''

if __name__=="__main__":
    # read PosT file:
    #（1）： load PosT file：
    #（2）： transform coordinates from system1 to system2
    #（3）： save PosT file or corresponding .txt file
    # origin_PosT_filename = "/media/mxx/MyPassport/zhangjiang-citysystem/@@2018-10-10-055907/POST/ReFinePOS.RePosT"
    origin_PosT_filename = "/home/mxx/data/Nanjing/轨迹/@@2024-09-03-050524_Camera1.PosC"
    transformed_txt_filename = "/home/mxx/data/Nanjing/轨迹/2024-09-03-050524_Camera1_XYZT.txt"
    io_PosT = IO_PosT()
    io_PosT.read_PosT(filename=origin_PosT_filename, pos_start_rowid=1)
    poses = io_PosT.get_poses()

    io_PosT.write_XYZT(transformed_txt_filename, post_list=poses, x_col=2, y_col=3, z_col=4, t_col=1)
