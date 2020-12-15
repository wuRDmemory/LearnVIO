import os
import os.path as osp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# dir_path = osp.dirname(osp.realpath(__file__))
# Hfile_path = osp.join(dir_path, 'Hmatrix.csv')

Hfile = pd.read_csv('/home/ubuntu/Hmatrix.csv')

H = Hfile.to_numpy()

H /= H.shape[0]

H_show = np.zeros(H.shape, np.float)
H_show[H!=0] = 1

plt.matshow(H_show)

plt.show()