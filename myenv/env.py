from myenv.base_env import BaseEnv
from myenv.utils import get_geom_id
import numpy as np

DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": -1,
    "distance": 4.0,
}


class MyRobotEnv(BaseEnv):
    def __init__(
        self,
        xml_path="/workspace/ros2_ws/src/RL-hockey/assets/main.xml",
        frame_skip: int = 5,
        default_camera_config=DEFAULT_CAMERA_CONFIG,
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
            xml_path,
            frame_skip,
            default_camera_config,
            joint_names,
            actuator_names,
            site_name,
            **kwargs,
        )
        self.racket_geom_id = get_geom_id(self.model, "racket_head")
        self.puck_geom_id = get_geom_id(self.model, "puck")
        self.init_site_pos = self.arm.get_site_pos().copy()

    def compute_reward(self, obs, action):
        # ── 1. ハイパーパラメータ（シンプル版） ────────────────
        HIT_REWARD          = 150.0  # ヒット時の大きな報酬
        MISS_PENALTY        = 100.0  # 打ち漏らしの大きな罰
        INTERCEPT_GAIN      = 75.0   # 迎撃地点にいることへの報酬
        
        # 予測関連
        DEFENSE_LINE_X      = -0.7   # 基本的な守備ライン
        MAX_PREDICTION_TIME = 1.5    # これ以上先の未来は予測しない

        # ── 2. 状態取得 ────────────────────────────────────
        puck_pos = self.puck.get_pos()
        puck_vel = self.puck.get_vel()
        ee_pos   = self.arm.get_site_pos()
        
        # ── 3. メインロジック：未来の迎撃点を予測 ───────────────
        reward = 0.0
        is_predictable = False
        predicted_y = 0.0
        time_to_impact = float('inf')

        # パックが十分な速度で自分に向かってきているか
        if puck_vel[0] < -0.1:
            # 守備ラインに到達するまでの時間を計算
            time_to_impact = (DEFENSE_LINE_X - puck_pos[0]) / puck_vel[0]
            
            # 現実的な時間内での予測か？
            if 0 < time_to_impact < MAX_PREDICTION_TIME:
                is_predictable = True
                predicted_y = puck_pos[1] + puck_vel[1] * time_to_impact

        # ── 4. 報酬の計算 ───────────────────────────────────
        # 【最重要】迎撃地点にいることへの「シェーピング報酬」
        if is_predictable:
            y_error = abs(ee_pos[1] - predicted_y)
            
            # 誤差がゼロに近いほど高い報酬を与える（ガウス関数を利用）
            # これが「どちらに動けば良いか」を示す、なだらかな坂（勾配）になる
            intercept_reward = INTERCEPT_GAIN * np.exp(-10.0 * y_error**2)
            
            # パックが近いほど、この報酬の重要度を上げる
            time_weight = 1.0 - (time_to_impact / MAX_PREDICTION_TIME)
            reward += intercept_reward * time_weight

        # 【イベント報酬】成功と失敗を明確に定義
        # ヒットした場合
        if self.check_puck_contact():
            reward += HIT_REWARD
            # ヒットしたら、そのエピソードでの「打ち漏らし」はなくなる
            self.miss_penalized = True 
        
        # 打ち漏らした場合
        if (puck_vel[0] < 0 
            and puck_pos[0] < DEFENSE_LINE_X
            and not getattr(self, 'miss_penalized', False)):
            reward -= MISS_PENALTY
            self.miss_penalized = True

        return reward


    def calc_site_puck_dist(self):
        site_pos = self.arm.get_site_pos()
        puck_pos = self.puck.get_pos()
        return np.linalg.norm(site_pos[:-1] - puck_pos)

    def check_puck_contact(self):
        """ラケットとパックの接触をチェック"""
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = contact.geom1
            geom2 = contact.geom2

            is_hit = (geom1 == self.racket_geom_id and geom2 == self.puck_geom_id) or (
                geom1 == self.puck_geom_id and geom2 == self.racket_geom_id
            )
            if is_hit:
                print("hit")
                return True
        return False
