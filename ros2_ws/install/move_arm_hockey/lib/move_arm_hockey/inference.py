#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from stable_baselines3 import SAC
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
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


class RlInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')

        # --- モデル読み込み ---
        self.declare_parameter('model_path', '/workspace/ros2_ws/src/crane_x7_ros/move_arm_hockey/src/sac_final.zip')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = SAC.load(model_path)
        self.get_logger().info("Model loaded successfully.")
        self.config = IKConfig('/workspace/ros2_ws/src/crane_x7_ros/move_arm_hockey/src/crane_x7.xml')
        self.control_freq = 0.02

        # --- 通信設定 ---
        # パブリッシュ先：JointState で関節角度コマンド
        self.joint_pub = self.create_publisher(JointState, 'arm_joint_command', 10)
        # サブスクライブ先：Float32MultiArray でパック中心位置
        self.puck_sub = self.create_subscription(
            Float32MultiArray,
            '/puck_nearest_center_m',
            self._puck_callback,
            10
        )
        self.arm_sub=self.create_subscription(
            JointState, '/joint_states',
            self._joint_callback_arm, 10
        )
        # 制御ループ: 50Hz
        self.timer = self.create_timer(self.control_freq, self._control_loop)

 
        # --- 状態変数 ---
        self.latest_puck_pos = None
        self.latest_puck_vel = None
        self.latest_arm_pos = None

    def _puck_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) != 2:
            self.get_logger().warning("Expected 2-element array for puck position.")
            return
        current_pos = np.array(data)
        if self.latest_puck_pos is not None:
            # Δt=1/30[s] 前提で速度
            self.latest_puck_vel = (current_pos - self.latest_puck_pos) * 30.0
        self.latest_puck_pos = current_pos
    
    def _joint_callback_arm(self, msg: JointState):
        self.latest_arm_pos=np.array(msg.position, dtype=np.float32)
    

    def _control_loop(self):
        if self.latest_puck_pos is None or self.latest_puck_vel is None:

            self.get_logger().warning("Puck data not ready.", throttle_duration_sec=5)
            return
        if self.latest_arm_pos is None:
            self.get_logger().warning("Arm data not ready.", throttle_duration_sec=5)
            return
        
        #外部から受け取る ここに値が入るようにする。
        self.puck_pos = self.latest_puck_pos
        self.arm_pos = self.latest_arm_pos[:7]
        self.puck_vel = self.latest_puck_vel
        
        obs = np.concatenate([
            self.puck_pos,
            self.arm_pos,
            self.puck_vel
        ]).astype(np.float32)

        # --- モデル推論: 関節角度列を出力 ---
        action, _ = self.model.predict(obs, deterministic=True)
        
        x=0.25*action[0]-0.75
        y=-0.4872*action[1]
        target_pos=np.array([x,y,0])
	# ラジアンの指令値->pubする直前に度数法に直す
        next_arm_pos=compute_ik_target_qpos(self.config, self.arm_pos, target_pos, self.control_freq)

        # --- JointState メッセージ化 ---
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
    'crane_x7_shoulder_fixed_part_pan_joint',
    'crane_x7_upper_arm_revolute_part_twist_joint',
    'crane_x7_upper_arm_revolute_part_rotate_joint',
    'crane_x7_lower_arm_fixed_part_joint',
    'crane_x7_shoulder_revolute_part_tilt_joint',
    'crane_x7_lower_arm_revolute_part_joint',
    'crane_x7_wrist_joint',
    'crane_x7_gripper_finger_a_joint'
        ]
        next_arm_pos_in_degree=np.rad2deg(next_arm_pos)
        self.get_logger().warning(f"指令値は{next_arm_pos}\n目標値{target_pos}", throttle_duration_sec=0.1)

        js.position = list(next_arm_pos_in_degree)
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RlInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
