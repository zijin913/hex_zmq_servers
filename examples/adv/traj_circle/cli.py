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
from hex_robo_utils import part2trans, trans2part
from hex_robo_utils import HexDynUtil as DynUtil

INIT_JOINT = np.array(
    [0.0, -0.0205679922, 2.57081467, -0.978840246, 0.0, 0.0],
    dtype=np.float64,
)
END_POSE = np.array(
    [0.0, 0.0, 0.187, 0.7071068, 0.0, -0.7071068, 0.0],
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


def create_traj_joint_arr(
    traj_center,
    traj_radius,
    traj_period,
    hz,
    dyn_util: DynUtil,
):
    center_pos = traj_center[:3]
    center_quat = traj_center[3:]
    trans_center_in_base = part2trans(center_pos, center_quat)

    circle_pos_list = []
    circle_quat_list = []
    circle_num = int(traj_period * hz)
    for i in range(circle_num):
        theta = 2 * np.pi * i / circle_num
        trans_pt_in_center = part2trans(
            np.array([
                0.0,
                traj_radius * np.sin(theta),
                traj_radius * np.cos(theta),
            ]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        trans_pt_in_base = trans_center_in_base @ trans_pt_in_center
        circle_pos, circle_quat = trans2part(trans_pt_in_base)
        circle_pos_list.append(circle_pos)
        circle_quat_list.append(circle_quat)

    traj_q_arr = np.zeros((circle_num, 6))
    traj_dq_arr = np.zeros((circle_num, 6))
    traj_q_arr[-1] = dyn_util.inverse_kinematics(
        tar_pose=(circle_pos_list[-1], circle_quat_list[-1]),
        start_q=INIT_JOINT,
        exit_eps=1e-4,
    )[1]
    for i in range(circle_num):
        traj_q_arr[i] = dyn_util.inverse_kinematics(
            (circle_pos_list[i], circle_quat_list[i]),
            traj_q_arr[i - 1],
        )[1]
    for i in range(circle_num):
        before_q = traj_q_arr[(i - 1 )% circle_num]
        after_q = traj_q_arr[(i + 1) % circle_num]
        traj_dq_arr[i] = 0.5 * (after_q - before_q) * hz
    return traj_q_arr, traj_dq_arr, circle_num


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        use_gripper = cfg["use_gripper"]
        ctrl_cfg = cfg["ctrl_cfg"]
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

    print(f"use_gripper: {use_gripper}")
    print("create_traj_joint_arr")
    traj_q_arr, traj_dq_arr, traj_num = create_traj_joint_arr(
        np.array(traj_cfg["traj_center"]),
        traj_cfg["traj_radius"],
        traj_cfg["traj_period"],
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
    while True:
        states_hdr, states = client.get_states()
        if states_hdr is not None:
            cur_q = states[:, 0]
            cur_dq = states[:, 1]
            arm_q = cur_q[:-1] if use_gripper else cur_q
            arm_dq = cur_dq[:-1] if use_gripper else cur_dq
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = c_mat @ arm_dq + g_vec

            ik_q = traj_q_arr[traj_idx]
            mid_q, interp_flag = interp_arm(
                cur_q,
                ik_q,
                grip_flag=False,
                use_gripper=use_gripper,
                err_limit=0.05,
            )
            if use_gripper:
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)

            tar_dq = np.zeros(mid_q.shape[0])
            if not interp_flag:
                tar_dq[:traj_dq_arr.shape[0]] = traj_dq_arr[traj_idx]
            cmds = np.zeros((mid_q.shape[0], 5))
            cmds[:, 0] = mid_q
            cmds[:, 1] = tar_dq
            cmds[:, 2] = tau_comp
            cmds[:, 3] = ctrl_cfg["mit_kp"]
            cmds[:, 4] = ctrl_cfg["mit_kd"]
            client.set_cmds(cmds)

        traj_idx = (traj_idx + 1) % traj_num
        rate.sleep()


if __name__ == '__main__':
    main()
