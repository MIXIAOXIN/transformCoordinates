'''
@Author: Xiaoxin Mi
@Email: xiaoxin.mi@whut.edu.cn; mixiaoxin_2012@163.com
@Brief: Read and write .txt files
'''

def load_txt(filename, split_flag=','):
    # opening the file in read mode
    with open(filename, "r") as my_file:
        # reading the file
        data_into_list= my_file.readlines()
    
    my_file.close()
    traj_pts = [[float(item) for item in traj.split(split_flag)] for traj in data_into_list]
    
    return traj_pts

def save_txt(filename, data_list, split_flag=', '):
    # write X, Y, Z, T into file
    with open(filename, "w") as f:
        # write xyzt in poses
        for pos in data_list:
            len_data = len(pos)
            for dt in len(len_data-1):
                str_out = str(pos[dt]) + split_flag 
            str_out += (str(pos[-1]) + '\n')
            f.write(str_out)
        f.close()