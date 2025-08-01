from myenv.ik import IKConfig, calc_ik_target_vel
from myenv.utils import get_actuator_id, get_joint_id, get_body_id, get_site_id

import numpy as np
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
from gymnasium import spaces
from gymnasium.spaces import Box
from typing import Dict, Union
from collections import deque
import mujoco


DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": -1,
    "distance": 4.0,
}


class PuckModel:
    def __init__(self, model, data, x_name, y_name):
        self.model = model
        self.data = data
        self.joint_ids = [get_joint_id(model, name) for name in [x_name, y_name]]

    def get_pos(self):
        qpos = self.data.qpos
        return qpos[self.joint_ids]

    def get_vel(self):
        qvel = self.data.qvel
        return qvel[self.joint_ids]

    def set_pos(self, pos):
        qpos = np.zeros_like(self.data.qpos)
        qpos[self.joint_ids] = pos
        return qpos

    def set_vel(self, vel):
        qvel = np.zeros_like(self.data.qvel)
        qvel[self.joint_ids] = vel
        return qvel


class ArmModel:
    def __init__(self, model, data, joint_names, actuator_names, site_name):
        self.model = model
        self.data = data
        self.joint_names = joint_names
        self.actuator_names = actuator_names
        self.site_name = site_name
        self.joint_ids = [get_joint_id(model, name) for name in joint_names]
        self.actuator_ids = [get_actuator_id(model, name) for name in actuator_names]
        self.site_id = get_site_id(self.model, site_name)
        self.dof = sorted(self.joint_ids)

    def get_pos(self):
        qpos = self.data.qpos
        return qpos[self.joint_ids]

    def get_vel(self):
        qvel = self.data.qvel
        return qvel[self.joint_ids]

    def set_pos(self, pos):
        qpos = np.zeros_like(self.data.qpos)
        qpos[self.joint_ids] = pos
        return qpos

    def set_vel(self, vel):
        qvel = np.zeros_like(self.data.qvel)
        qvel[self.joint_ids] = vel
        return qvel

    def get_site_pos(self):
        return self.data.site_xpos[self.site_id]

    def get_site_vel(self):
        """
        エンドエフェクタ（サイト）の線形速度をワールド座標系で取得します。
        """
        # 結果を格納するための6次元の配列（線形速度3D + 角速度3D）を初期化
        vel = np.zeros(6)
        
        # MuJoCoの関数を呼び出してサイトの速度を計算
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_SITE, # オブジェクトタイプをサイトに指定
            self.site_id,              # 速度を知りたいサイトのID
            vel,                       # 結果を格納する配列
            0,                         # 0: 速度をワールド座標系で計算
        )
        
        # 線形速度（最初の3要素）のみを返す
        return vel[:3]
    
    def set_ctrl(self, ctrl):
        result = np.zeros_like(self.data.ctrl)
        result[self.actuator_ids] = ctrl
        return ctrl


class BaseEnv(MujocoEnv):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
            "rgbd_tuple",
        ]
    }

    def __init__(
        self,
        xml_path="/workspace/RL-hockey2/assets/main.xml",
        frame_skip: int = 5,
        default_camera_config: Dict[str, Union[float, int]] = DEFAULT_CAMERA_CONFIG,
        joint_names=[
            # "crane_x7_gripper_finger_a_joint",
            # "crane_x7_gripper_finger_b_joint",
            "crane_x7_lower_arm_fixed_part_joint",
            "crane_x7_lower_arm_revolute_part_joint",
            "crane_x7_shoulder_fixed_part_pan_joint",
            "crane_x7_shoulder_revolute_part_tilt_joint",
            "crane_x7_upper_arm_revolute_part_rotate_joint",
            "crane_x7_upper_arm_revolute_part_twist_joint",
            "crane_x7_wrist_joint",
        ],
        actuator_names=[
            # "crane_x7_gripper_finger_a_joint",
            # "crane_x7_gripper_finger_b_joint",
            "crane_x7_lower_arm_fixed_part_joint",
            "crane_x7_lower_arm_revolute_part_joint",
            "crane_x7_shoulder_fixed_part_pan_joint",
            "crane_x7_shoulder_revolute_part_tilt_joint",
            "crane_x7_upper_arm_revolute_part_rotate_joint",
            "crane_x7_upper_arm_revolute_part_twist_joint",
            "crane_x7_wrist_joint",
        ],
        site_name="ee_site",
        **kwargs,
    ):
        super().__init__(
            model_path=xml_path,
            frame_skip=frame_skip,
            observation_space=None,
            render_mode="rgb_array",
            default_camera_config=default_camera_config,
            **kwargs,
        )
        self.puck_stop_threshold = 0.02  # これ以下の速度を「停止」と見なす
        self.puck_stop_duration = 50     # このステップ数だけ停止が続いたら終了
        self.stopped_counter = 0
        self.metadata["render_fps"] = int(np.round(1.0 / self.dt))
        self.action_delay_steps = 0
        self.action_buffer = deque(maxlen=self.action_delay_steps + 1)
        self.puck_launch_delay_steps = self.action_delay_steps
        self.episode_step_counter = 0
        NUM_ARM_JOINT = 7
        NUM_PUCK_JOINT = 2
        obs_dim = NUM_ARM_JOINT + NUM_PUCK_JOINT * 2
        self.observation_space = Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float64
        )

        ACTION_DIM = 2  # x, yの二自由度
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(ACTION_DIM,), dtype=np.float32
        )

        self.configure_puck()

        self.arm = ArmModel(
            self.model, self.data, joint_names, actuator_names, site_name
        )
        self.puck = PuckModel(self.model, self.data, "puck_x", "puck_y")
        self.config = IKConfig()

    def step(self, action):
        
        if self.episode_step_counter == self.puck_launch_delay_steps:
            # target_y = np.random.uniform(self.puck_y_range[0] * 0.9, self.puck_y_range[1] * 0.9)
            target_y = 0
            target_point = np.array([-1.1, target_y])
            direction_vector = target_point - self.puck_initial_pos
            center_angle = np.arctan2(direction_vector[1], direction_vector[0])
            angle_spread = np.pi / 4
            random_deviation = np.random.uniform(-angle_spread, angle_spread)
            final_angle = center_angle + random_deviation
            vx = self.puck_speed * np.cos(final_angle)
            vy = self.puck_speed * np.sin(final_angle)
            self.data.qvel[self.puck.joint_ids[0]] = vx
            self.data.qvel[self.puck.joint_ids[1]] = vy
        
        self.action_buffer.append(action)
        delayed_action = self.action_buffer[0]
        target_pos = np.array(
            [0.25 * delayed_action[0]-0.75, self.puck_y_range[0] * delayed_action[1], 0]
        )
        print(f"target_pos:{target_pos}")
        print(f"site_pos:{self.arm.get_site_pos()}")
        target_arm_vel = calc_ik_target_vel(
            self.model,
            self.data,
            target_pos,
            self.arm.site_id,
            self.arm.dof,
            [],
            self.config,
        )

        current_arm_joint_vel = self.data.qvel[self.arm.dof]
        # print(target_arm_vel, current_arm_joint_vel)
        velocity_error = target_arm_vel - current_arm_joint_vel
        torques = (
            self.config.Kp_joint_torque * velocity_error
            - self.config.Kd_joint_torque * current_arm_joint_vel
        )
        self.do_simulation(torques, self.frame_skip)
        obs = self.get_obs()
        reward = self.compute_reward(obs, delayed_action)
        done = self.is_done()
        truncated = done
        info = {}
        self.episode_step_counter += 1

        return obs, reward, done, truncated, info

    def is_done(self):
        """終了条件をチェック"""
        puck_pos = self.puck.get_pos()
        puck_vel= self.puck.get_vel()
        
        if puck_pos[0] < self.arm.get_site_pos()[0] - 0.1:
            return True
        if puck_vel[0] > 0.05 and self.episode_step_counter > self.puck_launch_delay_steps:
            self.stopped_counter += 1

        puck_speed = np.linalg.norm(puck_vel)
        if puck_speed < self.puck_stop_threshold and self.episode_step_counter > self.puck_launch_delay_steps:
            self.stopped_counter += 1
        # else:
        #     self.stopped_counter = 0
        
        if self.stopped_counter >= self.puck_stop_duration:
            return True

        return False

    def get_obs(self):
        puck_pos = self.puck.get_pos()
        puck_vel = self.puck.get_vel()
        arm_pos = self.arm.get_pos()
        return np.concatenate([puck_pos, arm_pos, puck_vel])

    def compute_reward(self, obs, action):
        raise NotImplementedError("Reward function is not implemented yet.")

    def reset_model(self):
        
    
        self.stopped_counter = 0
        self.episode_step_counter = 0
        qpos = np.array([0, 0, 0, -1, 0, -1, 0, -1, 0])
        qvel = self.init_qvel

        # qpos[self.puck.joint_ids[0]] = np.random.uniform(-0.1, self.puck_x_range[1])
        # qpos[self.puck.joint_ids[1]] = np.random.uniform(*self.puck_y_range*0.5)

        qpos[self.puck.joint_ids[0]] = -0.8
        qpos[self.puck.joint_ids[1]] = 0
        qvel[self.puck.joint_ids[0]] = 0.0
        qvel[self.puck.joint_ids[1]] = 0.0
        # print(f"DEBUG INFO: qpos shape={qpos.shape}, model nq={self.model.nq}")
        # print(f"DEBUG INFO: qvel shape={qvel.shape}, model nv={self.model.nv}")
        # print("--- Model Joint Information ---")
        # print(f"Total number of qpos elements (nq): {self.model.nq}")
        # print(f"Total number of joints (njnt): {self.model.njnt}")
        # print("Joint ID -> Joint Name Mapping:")
        # for i in range(self.model.njnt):
        #     joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
        #     # 各関節のqposインデックスを取得
        #     jnt_qposadr = self.model.jnt_qposadr[i]
        #     print(f"  Joint ID {i} (qpos index {jnt_qposadr}): {joint_name}")
        # print("-----------------------------")
        self.set_state(qpos, qvel)
        self.puck_initial_pos = self.puck.get_pos().copy()
        initial_ee_pos = self.arm.get_site_pos()
        initial_action_x = initial_ee_pos[0] / self.puck_x_range[0]
        initial_action_y = initial_ee_pos[1] / self.puck_y_range[0]
        initial_action = np.array([initial_action_x, initial_action_y])
        self.stopped_counter = 0

        self.action_buffer.clear()
        for _ in range(self.action_delay_steps + 1):
            self.action_buffer.append(initial_action)
        return self.get_obs()

    def configure_puck(self):
        self.puck_speed = 3.0
        self.table_surface_id = get_body_id(self.model, "table_surface")
        geom_indices = [
            j
            for j in range(self.model.ngeom)
            if self.model.geom_bodyid[j] == self.table_surface_id
        ]
        assert len(geom_indices) == 1
        x, y, _ = self.model.geom_size[geom_indices[0]]
        self.puck_x_range = np.array([-x, x]) * 0.8
        self.puck_y_range = np.array([-y, y]) * 0.8
