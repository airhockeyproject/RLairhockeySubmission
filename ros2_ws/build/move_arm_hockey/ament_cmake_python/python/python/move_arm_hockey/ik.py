import numpy as np
import mujoco
from typing import List, Tuple

class IKConfig:
    """IK計算に必要な設定と、MuJoCoのモデル・データを保持するクラス"""
    def __init__(self, model_path: str):
        # MuJoCoモデルとデータをロード
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # IK制御パラメータ
        self.Kp_pos = 50.0  # 位置制御Pゲイン
        self.Kp_ori = 81.0  # 姿勢制御Pゲイン
        self.lambda_dls = 0.1  # DLSのダンピング係数
        self.max_joint_vel = 3.0  # 目標関節速度のクリッピング値

        # ロボットのパラメータ
        self.ee_site_name = "ee_site" # 自身のモデルのエンドエフェクタ名に修正
        self.arm_joint_names = [ # 制御対象の関節名リスト
            'crane_x7_shoulder_fixed_part_pan_joint',
            'crane_x7_upper_arm_revolute_part_twist_joint',
            'crane_x7_upper_arm_revolute_part_rotate_joint',
            'crane_x7_lower_arm_fixed_part_joint',
            'crane_x7_shoulder_revolute_part_tilt_joint',
            'crane_x7_lower_arm_revolute_part_joint',
            'crane_x7_wrist_joint',
        ]
        # 関節名から各種インデックスを取得
        self.arm_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.arm_joint_names]
        self.arm_dof_qpos_indices = [self.model.jnt_qposadr[j_id] for j_id in self.arm_joint_ids]
        self.arm_dof_qvel_indices = [self.model.jnt_dofadr[j_id] for j_id in self.arm_joint_ids]
        self.wrist_joint_qvel_indices_in_nv = [] # 手首の減速が不要な場合は空リスト


def _calculate_ik_velocity(config: IKConfig, target_pos_world: np.ndarray) -> np.ndarray:
    """【内部関数】位置誤差のみから目標関節角速度を計算する"""
    # 目標姿勢 (固定)
    target_orientation_matrix_world = np.array([
        [-1, 0, 0], [0, 1, 0], [0, 0, -1],
    ])

    # 現在の手先位置・姿勢 (config.dataから取得)
    ee_site_id = mujoco.mj_name2id(config.model, mujoco.mjtObj.mjOBJ_SITE, config.ee_site_name)
    current_ee_pos = config.data.site_xpos[ee_site_id]
    current_ee_orientation_matrix = config.data.site_xmat[ee_site_id].reshape(3, 3)

    # 位置誤差に基づく目標速度 (P制御)
    pos_error = target_pos_world - current_ee_pos
    command_linear_vel = config.Kp_pos * pos_error

    # 姿勢誤差に基づく目標角速度 (P制御)
    target_quat = np.empty(4)
    current_quat = np.empty(4)
    mujoco.mju_mat2Quat(target_quat, target_orientation_matrix_world.flatten())
    mujoco.mju_mat2Quat(current_quat, current_ee_orientation_matrix.flatten())

    # --- ここからが修正箇所 ---
    # 1. 結果を格納する配列を準備
    neg_current_quat = np.empty(4)
    error_quat = np.empty(4)
    error_orientation_vec = np.empty(3)

    # 2. current_quatの逆クォータニオンを計算し、neg_current_quatに格納
    mujoco.mju_negQuat(neg_current_quat, current_quat)

    # 3. target_quat と neg_current_quat を乗算し、error_quat に格納
    mujoco.mju_mulQuat(error_quat, target_quat, neg_current_quat)

    # 4. 誤差クォータニオンから角速度ベクトルを計算
    mujoco.mju_quat2Vel(error_orientation_vec, error_quat, 1.0)
    # --- ここまでが修正箇所 ---

    command_angular_vel = config.Kp_ori * error_orientation_vec
    
    # 結合して6次元の目標速度ベクトルを作成
    task_space_desired_vel = np.concatenate([command_linear_vel, command_angular_vel])

    # ヤコビアンの計算
    ee_body_id = config.model.site_bodyid[ee_site_id]
    J_pos_site = np.zeros((3, config.model.nv))
    J_rot_body = np.zeros((3, config.model.nv))
    mujoco.mj_jacSite(config.model, config.data, J_pos_site, None, ee_site_id)
    mujoco.mj_jacBody(config.model, config.data, None, J_rot_body, ee_body_id)
    J_full = np.vstack([J_pos_site, J_rot_body])

    # DLS法でIKを解く
    try:
        A_matrix = J_full @ J_full.T + (config.lambda_dls**2) * np.eye(J_full.shape[0])
        solution = np.linalg.solve(A_matrix, task_space_desired_vel)
        qvel_des = J_full.T @ solution
    except np.linalg.LinAlgError:
        qvel_des = np.zeros(config.model.nv)

    return qvel_des

def compute_ik_target_qpos(
    config: IKConfig,
    real_joint_angles: np.ndarray,
    target_pos_world: np.ndarray,
    dt: float,
) -> np.ndarray:
    """
    現実の関節角度と目標座標から、次の目標関節角度を計算する。
    """
    # 1. 現実の状態でシミュレーションを更新 (順運動学)
    config.data.qpos[config.arm_dof_qpos_indices] = real_joint_angles
    config.data.qvel[:] = 0  # 速度はゼロと仮定
    mujoco.mj_forward(config.model, config.data)

    # 2. 更新された状態から目標関節角速度を計算
    target_joint_velocities_all = _calculate_ik_velocity(config, target_pos_world)
    
    # 制御対象の関節の速度のみを抽出・クリッピング
    target_joint_velocities_arm = target_joint_velocities_all[config.arm_dof_qvel_indices]
    clipped_velocities = np.clip(
        target_joint_velocities_arm, -config.max_joint_vel, config.max_joint_vel
    )

    # 3. 積分して次の目標関節角度を計算
    target_qpos = real_joint_angles + clipped_velocities * dt
    
    return target_qpos

if __name__ == "__main__":
    try:
        # 1. IKConfigのインスタンスを生成（ここでモデルが読み込まれる）
        MODEL_PATH = "/workspace/ros2_ws/src/crane_x7_ros/move_arm_hockey/src/crane_x7.xml"
        config = IKConfig(model_path=MODEL_PATH)
        

    except (FileNotFoundError, ValueError) as e:
        print(f"セットアップエラー: {e}")
        print("MODEL_PATHや関節名が正しいか確認してください。")
        exit()

    # --- 実行 ---
    real_angles = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0])
    target_pos = np.array([-0.8, 0, 0.2])

    print(f"現在の関節角度: {real_angles}")
    print(f"手先の目標座標: {target_pos}")
    
    # 統合関数を呼び出し
    next_target_joint_angles = compute_ik_target_qpos(config, real_angles, target_pos, 0.02)
    
    print("-" * 40)
    print(f"計算された次の目標関節角度:\n{next_target_joint_angles}")