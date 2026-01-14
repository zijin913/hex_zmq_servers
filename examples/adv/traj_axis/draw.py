#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-01-13
################################################################

import numpy as np
import matplotlib.pyplot as plt

traj_arr = np.load("traj_arr.npy")
print(f"traj_arr: {traj_arr.shape}")

traj_idx = range(traj_arr.shape[0])
plt.figure()
plt.plot(traj_idx, traj_arr[:, 0], label='joint_1')
plt.plot(traj_idx, traj_arr[:, 1], label='joint_2')
plt.plot(traj_idx, traj_arr[:, 2], label='joint_3')
plt.plot(traj_idx, traj_arr[:, 3], label='joint_4')
plt.plot(traj_idx, traj_arr[:, 4], label='joint_5')
plt.plot(traj_idx, traj_arr[:, 5], label='joint_6')
plt.legend()
plt.savefig("traj_axis.png")

pos_list = np.load("pos_list.npy")
print(f"pos_list: {pos_list.shape}")

pos_idx = range(pos_list.shape[0])
plt.figure()
plt.plot(pos_idx, pos_list[:, 0], label='x')
plt.plot(pos_idx, pos_list[:, 1], label='y')
plt.plot(pos_idx, pos_list[:, 2], label='z')
plt.legend()
plt.savefig("pos_list.png")