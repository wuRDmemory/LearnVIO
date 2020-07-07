import os

import argparse
import numpy as np
import pandas as pd

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_file(file_path):
    pose = []
    with open(file_path, 'r') as fp:
        for line in fp.readlines():
            elems = line.split('\t')
            pose.append([float(x) for x in elems])
    
    return pose

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description='arg parse')
    arg_parser.add_argument('-s',    type=str, default='', help='PATH/TO/YOUR/SLAM/OUTPUT')
    arg_parser.add_argument('-gt',   type=str, default='', help='PATH/TO/YOUR/GROUND/TRUTH')
    arg_parser.add_argument('-ldmk', type=str, default='', help='PATH/TO/YOUR/LANDMARK/OUTPUT')

    opt = arg_parser.parse_args()

    slam_path = opt.s
    gt_path   = opt.gt
    ldmk_path = opt.ldmk

    draw_slam = False
    draw_gt   = False
    draw_ldmk = False

    if slam_path:
        draw_slam = True
    
    if gt_path:
        draw_gt = True

    if ldmk_path:
        draw_ldmk = True

    print('Draw SLAM path')

    ''' draw slam path '''
    if draw_slam:
        poses = load_file(slam_path)
        poses = np.array(poses)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # ax.plot(poses[:, -3], poses[:, -2], poses[:, -1], 'r')

    if draw_ldmk:
        landmark = load_file(ldmk_path)
        landmark = np.array(landmark)

        ax.scatter(landmark[:, 1], landmark[:, 2], landmark[:, 3], 'b')

    ''' draw ground truth '''
    if draw_gt:
        gt_data = pd.read_csv(gt_path, skiprows=1)

        bgn_ts = poses[0, 0]
        end_ts = poses[-1, 0]
        bgn_i  = -1
        end_i  =  0
        for i in range(gt_data.size):
            ts = gt_data.iloc[i,0]/1e9
            if (bgn_i < 0 and ts > bgn_ts):
                print(ts)
                bgn_i = i
            
            if (ts > end_ts):
                print(ts)
                end_i = i
                break

        print(bgn_i, end_i)
        gt_data = gt_data.to_numpy()

        ax.plot(gt_data[bgn_i:end_i*3, 1], gt_data[bgn_i:end_i*3, 2], gt_data[bgn_i:end_i*3, 3], 'b')

    plt.show()
