"""
tracker.py
一维匀速(CV)模型卡尔曼滤波 + 目标跟踪器。

为什么需要：
    单线雷达只有 5Hz（一帧 200ms）。人走路 0.5m/s，
    200ms 就移动了 10cm —— 直接拿原始测量做控制必然滞后。
    卡尔曼既平滑噪声，又给出速度估计，用于 PD 的 D 项阻尼。

为什么跟踪 (d, θ) 而不是 (x, y)：
    故意不补偿本车运动。d 和 θ 正是要控制的量，
    本车前进导致 d 减小，本来就该反映进控制环。
    若换算到世界坐标再折算回来，反而多绕一圈、还引入 TF 延迟。
    代价是速度估计里混入本车速度分量，
    但这恰好让 D 项起阻尼作用，是有益的。
"""

import numpy as np


class CVKalman:
    """一维匀速模型卡尔曼滤波。

    状态 x = [位置, 速度]^T，测量 z = 位置。

    参数含义：
        q_pos / q_vel  过程噪声。调大 → 更信任测量、跟随更快但更抖；
                       调小 → 更平滑但滞后。
        r_meas        测量噪声。

    不引入 filterpy：四个 2x2 矩阵手写反而更好调试，
    也不用担心额外依赖装不上。
    """

    def __init__(self, q_pos=0.0015, q_vel=0.060, r_meas=0.010):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r = r_meas
        self.x = np.zeros(2)
        self.P = np.eye(2) * 1.0
        self.initialized = False

    def reset(self, z):
        """第一次观测：位置直接取测量值，速度置 0。"""
        self.x[0] = z
        self.x[1] = 0.0
        self.P = np.array([[self.r, 0.0],
                           [0.0, 1.0]])
        self.initialized = True

    def clear(self):
        self.initialized = False
        self.x = np.zeros(2)
        self.P = np.eye(2) * 1.0

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return

        # x = F x,  F = [[1, dt], [0, 1]]
        self.x[0] += self.x[1] * dt

        # P = F P F^T + Q
        p00, p01 = self.P[0, 0], self.P[0, 1]
        p10, p11 = self.P[1, 0], self.P[1, 1]

        f_p00 = p00 + dt * p10
        f_p01 = p01 + dt * p11
        f_p10 = p10
        f_p11 = p11

        fpf_00 = f_p00 + dt * f_p01
        fpf_01 = f_p01
        fpf_10 = f_p10 + dt * f_p11
        fpf_11 = f_p11
        # 对称化，消除浮点误差
        fpf_10 = fpf_01 = 0.5 * (fpf_01 + fpf_10)

        q00 = self.q_pos * dt * dt + self.q_vel * dt ** 4 / 4.0
        q01 = self.q_vel * dt ** 3 / 2.0
        q11 = self.q_vel * dt * dt

        self.P = np.array([
            [fpf_00 + q00, fpf_01 + q01],
            [fpf_10 + q01, fpf_11 + q11],
        ])

    def update(self, z):
        if not self.initialized:
            self.reset(z)
            return

        # H = [1, 0]
        y = z - self.x[0]                 # 新息
        s = self.P[0, 0] + self.r         # 新息协方差
        if s <= 0.0:
            return

        k = self.P[:, 0] / s              # 卡尔曼增益 (2,)
        self.x = self.x + k * y

        kh = np.outer(k, np.array([1.0, 0.0]))
        self.P = self.P - kh @ self.P
        self.P = 0.5 * (self.P + self.P.T)   # 强制对称

    @property
    def pos(self):
        return float(self.x[0])

    @property
    def vel(self):
        return float(self.x[1])


class TargetTracker:
    """把 (距离, 方位角) 两个卡尔曼打包，附带丢帧计数。"""

    def __init__(self, q_pos, q_vel, r_dist, r_angle):
        self.kf_d = CVKalman(q_pos, q_vel, r_dist)
        self.kf_a = CVKalman(q_pos, q_vel, r_angle)
        self.lost_frames = 0
        self.last_stamp = 0.0

    def reset(self):
        """清空全部状态（新目标）。"""
        self.kf_d.clear()
        self.kf_a.clear()
        self.lost_frames = 0
        self.last_stamp = 0.0

    def observe(self, dist, angle, stamp):
        """有测量时调用。

        返回 (滤波距离, 滤波角度, 距离变化率, 角度变化率)。
        """
        if self.kf_d.initialized and self.last_stamp > 0.0:
            dt = max(1e-3, stamp - self.last_stamp)
            self.kf_d.predict(dt)
            self.kf_a.predict(dt)

        self.kf_d.update(dist)
        self.kf_a.update(angle)
        self.last_stamp = stamp
        self.lost_frames = 0
        return (self.kf_d.pos, self.kf_a.pos,
                self.kf_d.vel, self.kf_a.vel)

    def predict_only(self, stamp):
        """这一帧没看到目标，用预测值顶一下。"""
        if self.kf_d.initialized and self.last_stamp > 0.0:
            dt = max(1e-3, stamp - self.last_stamp)
            self.kf_d.predict(dt)
            self.kf_a.predict(dt)
            self.last_stamp = stamp
        self.lost_frames += 1
        return self.kf_d.pos, self.kf_a.pos
