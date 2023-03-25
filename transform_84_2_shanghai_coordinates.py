'''
@ Author： Xiaoxin Mi
@ Email： mixiaoxin@whu.edu.cn, mixiaoxin_2012@163.com
@ Brief: This function aims for estimating 7 parameters in 3d coordinates transformation and evaluating the errors by
one more check points in two coordinate systems.
'''
import numpy as np
from data import CoorData
from cal_7_parameters import Cal7Parameters
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


    sigma0, sigmax = Conv_Model.evaluate()

    print('单位权中误差: %.3f' % (sigma0))
    print('参数中误差:')
    print(np.round(sigmax,3))
    (x2, y2, z2) = Conv_Model.coordinationConvert(source_test_points[0, 0], source_test_points[0, 1], source_test_points[0, 2], Args) + ave_tar
    print('模型预测结果: ')
    print('[(%.3f, %.3f, %.3f)]'%(x2, y2, z2))
    print('真实结果: ')
    print(target_test_points)