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
from hex_robo_utils import part2trans, trans2part, euler2rot, rot2quat
from hex_robo_utils import HexDynUtil as DynUtil

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


def create_traj_joint_arr(
    traj_center,
    traj_angle,
    traj_period,
    hz,
    dyn_util: DynUtil,
):
    traj_radius = np.sin(traj_angle)
    traj_distance = np.cos(traj_angle)
    center_pos = traj_center[:3]
    center_quat = traj_center[3:]
    trans_center_in_base = part2trans(center_pos, center_quat)

    circle_pos_list = []
    circle_quat_list = []
    circle_num = int(traj_period * hz)
    for i in range(circle_num):
        theta = 2 * np.pi * i / circle_num
        pitch = np.arctan2(-traj_radius * np.cos(theta), traj_distance)
        yaw = np.arctan2(traj_radius * np.sin(theta), traj_distance)
        rot = euler2rot(np.array([0, pitch, yaw]), format='xyz')
        trans_pt_in_center = part2trans(
            np.zeros(3),
            rot2quat(rot),
        )
        trans_pt_in_base = trans_center_in_base @ trans_pt_in_center
        circle_pos, circle_quat = trans2part(trans_pt_in_base)
        circle_pos_list.append(circle_pos)
        circle_quat_list.append(circle_quat)

    traj_joint_arr = np.zeros((circle_num, 6))
    traj_joint_arr[-1] = dyn_util.inverse_kinematics(
        tar_pose=(circle_pos_list[-1], circle_quat_list[-1]),
        start_q=INIT_JOINT,
        exit_eps=1e-4,
    )[1]
    for i in range(circle_num):
        traj_joint_arr[i] = dyn_util.inverse_kinematics(
            (circle_pos_list[i], circle_quat_list[i]),
            traj_joint_arr[i - 1],
        )[1]
    return traj_joint_arr, circle_num


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
    traj_arr, traj_num = create_traj_joint_arr(
        np.array(traj_cfg["traj_center"]),
        traj_cfg["traj_angle"],
        traj_cfg["traj_period"],
        1000,
        dyn_util,
    )
    print(f"traj_arr: {traj_arr.shape}")
    print(f"traj_num: {traj_num}")
    np.save("traj_arr.npy", traj_arr)

    if not wait_client_working(client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    traj_idx = 0
    rate = HexRate(1000)
    pos_list = []
    for _ in range(5 * traj_num):
    # while True:
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

            ik_q = traj_arr[traj_idx % traj_num]
            mid_q, _ = interp_arm(
                cur_q,
                ik_q,
                grip_flag=False,
                use_gripper=use_gripper,
                err_limit=0.05,
            )
            if use_gripper:
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)

            cmds = np.concatenate(
                (mid_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
            client.set_cmds(cmds)

        traj_idx = (traj_idx + 1) % traj_num
        rate.sleep()
    
    pos_list = np.array(pos_list)
    print(f"pos_list: {pos_list.shape}")
    np.save("pos_list.npy", pos_list)

if __name__ == '__main__':
    main()
