#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
L6Y GP100 MuJoCo 仿真测试脚本 - 关节位置控制

运行方式:
    cd hex_zmq_servers
    source .venv/bin/activate
    python hex_zmq_servers/mujoco/archer_l6y/test_sim.py
"""

import os
import mujoco
from mujoco import viewer


def main():
    model_path = os.path.join(os.path.dirname(__file__), "model/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 加载 home 姿态
    keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)

    # 启动 viewer，直接控制 qpos（关节位置）
    viewer.launch(model, data)


if __name__ == "__main__":
    main()
