'''
@ Author: Xiaoxin Mi
'''
import os
import numpy as np
from data import CoorData
from cal_7_parameters import Cal7Parameters
from IO_PosT import IO_PosT



'''
@ Brief: This function aims for estimating 7 parameters in 3d coordinates transformation and evaluating the errors by 
one more check points in two coordinate systems.
'''

if __name__=="__main__":
    # several source points and corresponding target points
    data_store = CoorData()
    data_store.set_84_to_shanghai_data()
    source_points, target_points = data_store.get_data()
    # 检查点坐标
    source_test_points_o, target_test_points = data_store.get_test_data()

    # 归一化处理便于模型的快速迭代
    ave_src = np.mean(np.array(source_points), axis=0)
    ave_tar = np.mean(np.array(target_points), axis=0)
    source_points -= ave_src
    target_points -= ave_tar
    # 归一化处理
    source_test_points = np.array(source_test_points_o - ave_src)

    # 打印七参数
    error_thre = 1e-13
    Conv_Model=Cal7Parameters(error_thre)
    Args = Conv_Model.calculate(source_points, target_points)
    print("Args:")
    print(np.round(Args, 3))


    print('source test pints.shape: ', source_test_points.shape)
    sigma0, sigmax = Conv_Model.evaluate()
    print('单位权中误差: %.3f' % (sigma0))
    print('参数中误差:')
    print(np.round(sigmax,3))
    (x2, y2, z2) = Conv_Model.coordinationConvert(source_test_points[0, 0], source_test_points[0, 1], source_test_points[0,2], Args)
    print('ave_tar.shape: ', ave_tar.shape)
    x2 += ave_tar[0]
    y2 += ave_tar[1]
    z2 += ave_tar[2]
    print('模型预测结果: ')
    print('[(%.3f, %.3f, %.3f)]' % (x2, y2, z2))
    print('真实结果: ')
    print(target_test_points)

    # 2 points
    source_test_pts = np.array(source_test_points)
    source_test_pts = source_test_pts.repeat(2, axis=0)
    source_test_pts = np.transpose(source_test_pts)
    ave_tar_matrix = ave_tar.reshape(3,1).repeat(source_test_pts.shape[1], axis=1)
    pts_converted = Conv_Model.convertCoorninates(Args, source_test_pts)
    pts_converted += ave_tar_matrix
    print("many predicted results: ", pts_converted)

    # read PosT file:
    #（1）： load PosT file：
    #（2）： transform coordinates from system1 to system2
    #（3）： save PosT file or corresponding .txt file
    # origin_PosT_filename = "/media/mxx/MyPassport/zhangjiang-citysystem/@@2018-10-10-055907/POST/ReFinePOS.RePosT"
    origin_PosT_filename = "/home/mxx/Desktop/ReFinePOS.RePosT"
    io_PosT = IO_PosT()
    io_PosT.read_PosT(filename=origin_PosT_filename)
    poses = io_PosT.get_poses()
    pos_origin = np.zeros((len(poses), 4))
    pos_transformed = []
    for idx, pt in enumerate(poses):
        gps_t = pt[1]
        pt_x = pt[3]
        pt_y = pt[2]
        pt_z = pt[4]
        # print('raw pos line: ', pt)
        # print("this pos raw: ", pt_x, pt_y, pt_z, gps_t)
        pos_origin[idx, :] = np.array([np.longdouble(pt_x), np.longdouble(pt_y), np.longdouble(pt_z), np.longdouble(gps_t)])
        # print("this pos is: ", pos_origin[idx, :])
    # first translation;
    pos_origin[:, :3] -= ave_src
    pos_transformed = Conv_Model.convertCoorninates(Args, np.transpose(pos_origin[:, :3]))
    pos_transformed += ave_tar.reshape(3,1).repeat(pos_origin.shape[0], axis=1)  # add target translation
    pos_transformed = np.concatenate((pos_transformed, np.transpose(np.expand_dims(pos_origin[:, 3], axis=0).transpose())), axis=0)
    pos_transformed = np.transpose(pos_transformed)

    new_pos = poses.copy()
    for idx, pt in enumerate(pos_transformed):
        new_pos[idx, 3] = pt[0]  # x
        new_pos[idx, 2] = pt[1]  # y
        new_pos[idx, 4] = pt[2]  # z
    print("pos_transformed shape: ", pos_transformed.shape)
    print("random pos: ", pos_transformed[-1, :])

    filestem, filesuffix = os.path.splitext(origin_PosT_filename)
    print(filestem, filesuffix)
    transformed_PosT_filename = filestem + '_shanghai' + filesuffix
    transformed_txt_filename = filestem + '_shanghai.txt'
    io_PosT.write_PosT(transformed_PosT_filename, post_list=new_pos)
    io_PosT.write_XYZT(transformed_txt_filename, post_list=pos_transformed)




