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
               dofs: dict = None,
               err_limit=0.05):
    mid_joint = np.zeros(dofs["sum"])
    mid_joint[:dofs["robot_arm"]], interp_flag = interp_joint(
        cur_q[:dofs["robot_arm"]],
        tar_joint,
        err_limit=err_limit,
    )
    if dofs["robot_gripper"] is not None:
        mid_joint[-dofs["robot_gripper"]:], _ = interp_joint(
            cur_q[-dofs["robot_gripper"]:],
            1.33 if grip_flag else 0.2,
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

    joint_num = dyn_util.get_joint_num()
    traj_q_arr = np.zeros((circle_num, joint_num))
    traj_q_arr[-1] = dyn_util.inverse_kinematics(
        tar_pose=(circle_pos_list[-1], circle_quat_list[-1]),
        start_q=INIT_JOINT,
        exit_eps=1e-4,
    )[1]
    for i in range(circle_num):
        traj_q_arr[i] = dyn_util.inverse_kinematics(
            tar_pose=(circle_pos_list[i], circle_quat_list[i]),
            start_q=traj_q_arr[i - 1],
            exit_eps=1e-4,
        )[1]

    cur_dq_arr = np.zeros((circle_num, joint_num))
    for i in range(circle_num):
        before_q = traj_q_arr[(i - 1) % circle_num]
        after_q = traj_q_arr[(i + 1) % circle_num]
        cur_dq_arr[i] = 0.5 * (after_q - before_q) * hz

    weight_np = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
    weight_np = weight_np / weight_np.sum()
    idx_arr = np.arange(weight_np.shape[0]) - weight_np.shape[0] // 2
    traj_dq_arr = np.zeros((circle_num, joint_num))
    for i in range(circle_num):
        idx_vec = idx_arr + i
        vel_vec = cur_dq_arr[idx_vec % circle_num]
        traj_dq_arr[i] = np.dot(weight_np, vel_vec)

    return traj_q_arr, traj_dq_arr, circle_num


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
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

    print("create_traj_joint_arr")
    traj_q_arr, traj_dq_arr, traj_num = create_traj_joint_arr(
        np.array(traj_cfg["traj_center"]),
        traj_cfg["traj_angle"],
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

    dof_arr = client.get_dofs()
    dofs = {
        "robot_arm": int(dof_arr[0]),
        "robot_gripper": int(dof_arr[1]) if len(dof_arr) > 1 else None,
        "sum": int(dof_arr.sum()),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    traj_idx = 0
    init_flag = True
    init_limit = 0.03
    runtime_limit = 0.2
    rate = HexRate(1000)
    while True:
        states_hdr, states = client.get_states()
        if states_hdr is not None:
            cur_q = states[:, 0]
            cur_dq = states[:, 1]
            arm_q = cur_q[:dofs["robot_arm"]]
            arm_dq = cur_dq[:dofs["robot_arm"]]

            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = np.zeros(dofs["sum"])
            tau_comp[:dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

            ik_q = traj_q_arr[traj_idx]
            mid_q, interp_flag = interp_arm(
                cur_q,
                ik_q,
                grip_flag=False,
                dofs=dofs,
                err_limit=init_limit if init_flag else runtime_limit,
            )

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
