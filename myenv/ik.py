import numpy as np
import mujoco
from typing import List


class IKConfig:
    # IK制御パラメータ (タスクスペース用)
    Kp_pos = 50.0  # 位置制御Pゲイン
    Kd_pos = 0.4  # 位置制御Dゲイン
    Kp_ori = 81.0  # 姿勢制御Pゲイン
    Kd_ori = 0.05  # 姿勢制御Dゲイン
    wrist_weight_factor = 100000
    lambda_dls = 0.1  # DLSのダンピング係数
    max_joint_vel = 3.0  # IKが計算する目標関節速度のクリッピング値

    # 関節トルク制御用PDゲイン (ジョイントスペース用)
    Kp_joint_torque = 100.0  # 関節Pゲイン (目標速度誤差に対して)
    Kd_joint_torque = 1.1  # 関節Dゲイン (現在速度に対して、ダンピング)


def calc_ik_target_vel(
    model,
    data,
    target_pos_world: np.ndarray,
    ee_site_id: int,
    arm_dof_qvel_indices: List[int],
    wrist_joint_qvel_indices_in_nv: List[int],
    config: IKConfig,
) -> np.ndarray:
    target_orientation_matrix_world = np.array(
        [
            [-1, 0.0, 0],
            [0.0, 1.0, 0.0],
            [0, 0.0, -1],
        ]
    )
    ee_body_id = model.site_bodyid[ee_site_id]
    current_ee_pos = data.site_xpos[ee_site_id]
    current_ee_orientation_matrix = data.site_xmat[ee_site_id].reshape(3, 3)

    J_pos_site = np.zeros((3, model.nv))
    J_rot_body = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, J_pos_site, None, ee_site_id)
    mujoco.mj_jacBody(model, data, None, J_rot_body, ee_body_id)
    J_full = np.vstack([J_pos_site, J_rot_body])

    current_ee_linear_vel = J_pos_site @ data.qvel[: model.nv]
    current_ee_angular_vel = J_rot_body @ data.qvel[: model.nv]

    pos_error = target_pos_world - current_ee_pos
    target_ee_linear_vel_world = np.zeros(3)
    command_linear_vel = (
        target_ee_linear_vel_world
        + config.Kp_pos * pos_error
        - config.Kd_pos * current_ee_linear_vel
    )

    target_quat_arr = np.empty(4)
    current_quat_arr = np.empty(4)
    mujoco.mju_mat2Quat(target_quat_arr, target_orientation_matrix_world.flatten())
    mujoco.mju_mat2Quat(current_quat_arr, current_ee_orientation_matrix.flatten())
    conj_current_quat_arr = current_quat_arr.copy()
    conj_current_quat_arr[1:] *= -1.0
    error_quat_arr = np.empty(4)
    error_orientation_vec = np.empty(3)
    mujoco.mju_mulQuat(error_quat_arr, target_quat_arr, conj_current_quat_arr)
    mujoco.mju_quat2Vel(error_orientation_vec, error_quat_arr, 1.0)

    target_ee_angular_vel_world = np.zeros(3)
    command_angular_vel = (
        target_ee_angular_vel_world
        + config.Kp_ori * error_orientation_vec
        - config.Kd_ori * current_ee_angular_vel
    )
    task_space_desired_vel = np.concatenate([command_linear_vel, command_angular_vel])

    # --- WDLSのための重み行列設定 ---
    W_inv = np.eye(model.nv)  # まずは単位行列で初期化 (全関節の重みは同じ)
    if len(wrist_joint_qvel_indices_in_nv) > 0:  # 手首のインデックスが指定されていれば
        W_inv[wrist_joint_qvel_indices_in_nv, wrist_joint_qvel_indices_in_nv] = (
            config.wrist_weight_factor
        )
    # ---------------------------------

    try:
        JW = J_full @ W_inv
        A_matrix_wdls = JW @ J_full.T + (config.lambda_dls**2) * np.eye(J_full.shape[0])
        solution = np.linalg.solve(A_matrix_wdls, task_space_desired_vel)
        qvel_des_all_dof = W_inv @ J_full.T @ solution
    except np.linalg.LinAlgError:
        print("IK Warning: Jacobian inversion failed. Setting qvel_des to zero.")
        qvel_des_all_dof = np.zeros(model.nv)

    target_arm_joint_velocities_unclipped = qvel_des_all_dof[arm_dof_qvel_indices]
    target_arm_joint_velocities_clipped = np.clip(
        target_arm_joint_velocities_unclipped,
        -config.max_joint_vel,
        config.max_joint_vel,
    )
    return target_arm_joint_velocities_clipped


if __name__ == "__main__":
    import time
    import sys

    sys.path.append("/workspace/RL-hockey2")
    from myenv.env import MyRobotEnv

    env = MyRobotEnv()
    env.reset()
    frames = []
    start = time.perf_counter()
    for _ in range(1000):
        obs, *_ = env.step(np.array([1, 0]))
        frames.append(env.render())
    print("IK計算時間", time.perf_counter() - start)
