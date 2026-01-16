#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHexarmClient,
)
from hex_robo_utils import HexDynUtil as DynUtil
from hex_robo_utils import HexPlotUtilPlotJuggler as HexPlotUtil

INIT_JOINT = np.array(
    [0.0, -0.0205679922, 2.57081467, -0.978840246, 0.0, 0.0],
    dtype=np.float64,
)
END_POSE = np.array(
    [0.0, 0.0, 0.083, 0.7071068, 0.0, -0.7071068, 0.0],
    dtype=np.float64,
)


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        if client.is_working():
            if hasattr(client, "seq_clear"):
                client.seq_clear()
            return True
        else:
            time.sleep(0.1)
    return False


def interp_joint(cur_q, tar_joint, err_limit=0.05):
    err = tar_joint - cur_q
    max_err_fab = np.fabs(err).max()
    if max_err_fab < err_limit:
        return tar_joint, False
    else:
        err_norm = err / max_err_fab
        return cur_q + err_norm * err_limit, True


def interp_arm(cur_q,
               tar_joint,
               grip_flag=True,
               use_gripper=True,
               err_limit=0.05):
    mid_joint = np.zeros(7 if use_gripper else 6)
    if use_gripper:
        mid_joint[:-1], interp_flag = interp_joint(
            cur_q[:-1],
            tar_joint,
            err_limit=err_limit,
        )
        mid_joint[-1], _ = interp_joint(
            cur_q[-1],
            1.33 if grip_flag else 0.2,
            err_limit=err_limit,
        )
    else:
        mid_joint, interp_flag = interp_joint(
            cur_q,
            tar_joint,
            err_limit=err_limit,
        )
    return mid_joint, interp_flag


def calc_time_vec(t):
    t = np.asarray(t)
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    return np.stack([np.ones_like(t), t, t2, t3, t4, t5], axis=0)


def calc_coeffs(start, end, t):
    # Matrix A
    t_vec = calc_time_vec(t)
    mat_a = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, t_vec[1], t_vec[2], t_vec[3], t_vec[4], t_vec[5]],
        [0, 1, 2 * t_vec[1], 3 * t_vec[2], 4 * t_vec[3], 5 * t_vec[4]],
        [0, 0, 2, 6 * t_vec[1], 12 * t_vec[2], 20 * t_vec[3]],
    ])
    inv_a = np.linalg.inv(mat_a)

    # Matrix B
    mat_b = np.zeros((6, len(start)))
    mat_b[0, :] = start.copy()
    mat_b[3, :] = end.copy()

    # shape: (6, joint_num)
    return inv_a @ mat_b


def create_traj_joint_arr(
    traj_poses,
    traj_move,
    traj_stop,
    hz,
    dyn_util: DynUtil,
):
    pt_pos_list = []
    pt_quat_list = []
    pose_num = traj_poses.shape[0]
    for pt_idx in range(pose_num):
        pt_pose = traj_poses[pt_idx]
        pt_pos = pt_pose[:3].copy()
        pt_quat = pt_pose[3:].copy()
        pt_pos_list.append(pt_pos)
        pt_quat_list.append(pt_quat)
    pt_pos_list.append(pt_pos_list[0].copy())
    pt_quat_list.append(pt_quat_list[0].copy())

    stop_num = int(traj_stop * hz)
    joint_num = dyn_util.get_joint_num()
    traj_q_list, traj_dq_list = [], []
    for traj_idx in range(pose_num):
        start_q = dyn_util.inverse_kinematics(
            tar_pose=(pt_pos_list[traj_idx], pt_quat_list[traj_idx]),
            start_q=INIT_JOINT,
            exit_eps=1e-4,
        )[1]
        end_q = dyn_util.inverse_kinematics(
            tar_pose=(pt_pos_list[(traj_idx + 1) % pose_num],
                      pt_quat_list[(traj_idx + 1) % pose_num]),
            start_q=start_q,
            exit_eps=1e-4,
        )[1]

        # stop
        traj_q_list.append(np.ones((stop_num, joint_num)) * start_q)
        traj_dq_list.append(np.zeros((stop_num, joint_num)))

        # move
        # max accel: 2.5 rad/s^2
        delta_q = np.abs(end_q - start_q)
        delta_q_time = np.max(2.0 * delta_q / np.sqrt(4))
        move_time = traj_move
        if delta_q_time > traj_move:
            move_time = delta_q_time
            hex_log(HEX_LOG_LEVEL["warn"],
                    f"traj_move is too small, set to {move_time}")
        move_num = int(move_time * hz) + 1
        move_time = move_num / hz

        coeff_mat = calc_coeffs(start_q, end_q, move_time)
        t_arr = np.linspace(0, move_time, move_num)
        t_vec_arr = calc_time_vec(t_arr)
        traj_q = (coeff_mat[0, :, None] +
                  coeff_mat[1, :, None] * t_vec_arr[1] +
                  coeff_mat[2, :, None] * t_vec_arr[2] +
                  coeff_mat[3, :, None] * t_vec_arr[3] +
                  coeff_mat[4, :, None] * t_vec_arr[4] +
                  coeff_mat[5, :, None] * t_vec_arr[5]).T
        traj_dq = (coeff_mat[1, :, None] +
                   2 * coeff_mat[2, :, None] * t_vec_arr[1] +
                   3 * coeff_mat[3, :, None] * t_vec_arr[2] +
                   4 * coeff_mat[4, :, None] * t_vec_arr[3] +
                   5 * coeff_mat[5, :, None] * t_vec_arr[4]).T
        traj_q_list.append(traj_q)
        traj_dq_list.append(traj_dq)

    traj_q_arr = np.concatenate(traj_q_list, axis=0)
    traj_dq_arr = np.concatenate(traj_dq_list, axis=0)
    return traj_q_arr, traj_dq_arr, traj_q_arr.shape[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        use_gripper = cfg["use_gripper"]
        mit_kp = np.array(cfg["ctrl_cfg"]["mit_kp"])
        mit_kd = np.array(cfg["ctrl_cfg"]["mit_kd"])
        traj_cfg = cfg["traj_cfg"]
        net_config = cfg["net"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    client = HexRobotHexarmClient(net_config=net_config)
    dyn_util = DynUtil(
        model_path=model_path,
        last_link=last_link,
        end_pose=END_POSE,
    )
    plot_util = HexPlotUtil()

    print(f"use_gripper: {use_gripper}")
    print("create_traj_joint_arr")
    traj_q_arr, traj_dq_arr, traj_num = create_traj_joint_arr(
        np.array(traj_cfg["traj_poses"]),
        traj_cfg["traj_move"],
        traj_cfg["traj_stop"],
        1000,
        dyn_util,
    )
    print(f"traj_q_arr: {traj_q_arr.shape}")
    print(f"traj_dq_arr: {traj_dq_arr.shape}")
    print(f"traj_num: {traj_num}")

    if not wait_client_working(client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    traj_idx = 0
    rate = HexRate(1000)
    pos_list = []
    init_flag = True
    init_limit = 0.03
    runtime_limit = 0.1
    while True:
        states_hdr, states = client.get_states()
        if states_hdr is not None:
            cur_q = states[:, 0]
            cur_dq = states[:, 1]
            arm_q = cur_q[:-1] if use_gripper else cur_q
            arm_dq = cur_dq[:-1] if use_gripper else cur_dq

            pos, _ = dyn_util.forward_kinematics(arm_q)[-1]
            pos_list.append(pos.copy())

            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = c_mat @ arm_dq + g_vec

            ik_q = traj_q_arr[traj_idx]
            mid_q, interp_flag = interp_arm(
                cur_q,
                ik_q,
                grip_flag=False,
                use_gripper=use_gripper,
                err_limit=init_limit if init_flag else runtime_limit,
            )
            if use_gripper:
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)

            tar_dq = np.zeros(mid_q.shape[0])
            if not interp_flag:
                if init_flag:
                    init_flag = False
                    print("init finished")
                tar_dq[:traj_dq_arr.shape[1]] = traj_dq_arr[traj_idx]
            cmds = np.zeros((mid_q.shape[0], 5))
            cmds[:, 0] = mid_q
            cmds[:, 1] = tar_dq
            cmds[:, 2] = tau_comp
            cmds[:, 3] = mit_kp
            cmds[:, 4] = mit_kd
            client.set_cmds(cmds)

            plot_util.add_arr(
                name="jnt",
                data=states[:, :-1],
                labels=["q", "dq"],
            )
            pos, quat = dyn_util.forward_kinematics(arm_q)[-1]
            pose_dict = {
                "pos": pos.tolist(),
                "quat": quat.tolist(),
            }
            plot_util.add_data(
                name="pose",
                data=pose_dict,
            )
            plot_util.send_data(clear=True)

        traj_idx = (traj_idx + 1) % traj_num
        rate.sleep()


if __name__ == '__main__':
    main()
