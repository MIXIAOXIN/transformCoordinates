'''
@Author: Xiaoxin Mi
@E-mail: mixiaoxin@whu.edu.cn
@Thanks:
'''
import os
import numpy as np

# 根据旋转平移矩阵坐标转换公式来列立系数方程定义函数返回系数矩阵。
# 使用4个同名点的坐标转换公式作为误差方程进行最小二乘平差，根据坐标转换公式来列立系数方程 Wx=b


class CalQRParameters():
    def __init__(self,  error_thre=1e-10):
        # 组成法方程 (W’W) x = (W’b) 并利用最小二乘求解x
        # First 4 parameters are quaternions, last 3 are translations
        self.error_thre = error_thre
        self.Args = np.array([0, 0, 0, 0, 0, 0, 0], dtype='float64')
        self.parameters = np.array([1, 1, 1, 1, 1, 1, 1])
        self.W = None
        self.b = None
        self.qxx = None


    def calculate(self, source_points, target_points):
        # 当七参数的误差值之和大于1e-10时，迭代运算得到更精确的结果
        while np.fabs(np.array(self.parameters)).sum() > self.error_thre:
            self.W = self.points2W(source_points, self.Args)
            self.b = self.points2b(source_points, target_points, self.Args)
            self.qxx = np.linalg.inv(np.dot(self.W.transpose(), self.W))
            self.parameters = np.dot(np.dot(self.qxx, self.W.transpose()), self.b)
            self.Args += self.parameters
        return self.Args

    def evaluate(self):
        # 单位权标准差，即所得模型的坐标精度
        sigma0 = np.sqrt((self.b * self.b).sum() / 2)
        # 参数标准差，即所得模型的七参数精度
        sigmax = sigma0 * np.sqrt(np.diag(self.qxx))

        return  sigma0, sigmax
    '''
    rotation matrix = 
    [ q0^2 + q1^2 - q2^2 - q3^2, 2(q1q2-q0q3),               2(q1q3+q0q2)],
    [2(q1q2 + q0q3),             q0^2 - q1^2 + q2^2 - q3^2,  2(q2q3-q0q1)],
    [2(q1q3 - q0q2),             2(q2q3+q0q1),               q0^2 - q1^2 - q2^2 + q3^2]
    ]
    
    and translation verctor
    '''
    def point2matrix(self, x, y, z, args):
        array = [
            [(args[0]*args[0]+args[1]*args[1]-args[2]*args[2]-args[3]*args[3]) , 2*(args[1]*args[2]-args[0]*args[3]), 2*(args[1]*args[3]+args[0]*args[2]), args[4]],
            [2*(args[1]*args[2]+args[0]*args[3]), args[0]*args[0]-args[1]*args[1]+args[2]*args[2]-args[3]*args[3], 2*(args[2]*args[3]-args[0]*args[1]), args[5]],
            [2*(args[1]*args[3]-args[0]*args[2]), 2*(args[2]*args[3]+args[0]*args[1]), args[0]*args[0]-args[1]*args[1]-args[2]*args[2]+args[3]*args[3], args[6]],
            [0, 0, 0, 1]
        ]
        return np.array(array)

    # 定义函数： points2W
    #      通过同名点序列列立误差方程B系数阵的整体
    #       x, y, z： 同名点序列
    #       args: 七参数误差值[q0, q1, q2, q3, t_x, t_y, t_z]
    #       返回值： W系数阵

    def points2W(self, points, args):
        big_mat = None
        for (x, y, z) in points:
            mat = self.point2matrix(x, y, z, args)
            if big_mat is None:
                big_mat = mat
            else:
                big_mat = np.concatenate((big_mat, mat))

        return big_mat

    # 定义函数： points2b
    #       通过同名点坐标转换关系列立误差方程B系数阵的整体
    #       x, y, z： 同名点的原坐标和目标坐标对组成的序列
    #       args: 七参数误差值[Delta_X, Delta_Y, Delta_Z, theta_x, theta_y, theta_z, m]
    #       返回值： b系数阵

    def points2b(self, source, target, args):
        big_mat = [0] * len(source) * 3

        for i, ((x1, y1, z1), (x2, y2, z2)) in enumerate(zip(source, target)):
            (x0, y0, z0) = self.coordinationConvert(x1, y1, z1, args)
            big_mat[3 * i + 0] = x2 - x0
            big_mat[3 * i + 1] = y2 - y0
            big_mat[3 * i + 2] = z2 - z0

        return np.array(big_mat).transpose()

    def coordinationConvert(self, x1, y1, z1, args):
        x2 = args[0] + (1 + args[6]) * (x1 + args[5] * y1 - args[4] * z1)
        y2 = args[1] + (1 + args[6]) * (-args[5] * x1 + y1 + args[3] * z1)
        z2 = args[2] + (1 + args[6]) * (args[4] * x1 - args[3] * y1 + z1)
        return (x2, y2, z2)

