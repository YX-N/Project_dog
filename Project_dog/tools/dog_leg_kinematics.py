"""
机器狗左前腿运动学模型：足端位置  <->  3 个电机角
（几何参数从 Ljsz-1.2-06-00-00.STEP 提取）
====================================================================

用法
----
    from dog_leg_kinematics import LegModel
    leg = LegModel()
    q, info = leg.foot_to_motor([200, 120.5, -250])        # 足端位置 (mm, 髋坐标系) -> 3 电机角 (rad)
    Q, infos = leg.foot_to_motor(traj_Nx3)                 # 轨迹
    p = leg.motor_to_foot(q)                               # 电机角 -> 足端位置
    J = leg.jacobian(theta)                                # 足端速度 = J @ 关节角速度

机构（从装配体和螺钉连接关系判断）
--------------------------------
机身 = HAA 电机定子
  --HAA 电机(髋关节电机组)，绕机身 x 轴（前后方向）-->  胯关节支架 + HFE 电机定子
  --HFE 电机(左前胯关节电机组)，绕 y 轴（左右方向）-->  大腿（含连接座、膝电机定子、大腿板）
  --膝电机(左前膝关节电机组)：输出轴 18 齿 -> 大腿上 26 齿（模数 1.5，中心距 33）
      -> 26 齿上的曲柄 r=22 -> 平行四边形连杆(180.2) -> 小腿
  小腿相对大腿的转角 = 曲柄相对大腿的转角 = -(18/26) × 膝电机相对大腿的转角
三个关节都是电机直接给出的角度关系（无并联闭链），HAA 轴与 HFE 轴相交于髋中心 H。

坐标系（髋坐标系，ROS REP-103：x 前、y 左、z 上；单位 mm、rad）
------------------------------------------------------------------
原点 = 髋中心 H（HAA 轴与 HFE 轴交点，CAD 坐标 (-270.9, 238.0, 0)）
x = CAD -X，y = CAD +Z，z = CAD +Y。
"左前腿"位于 CAD +Z 一侧，由此推定 CAD +Z 为左、CAD -X 为前；若实际朝向不同，只需改 CAD2HIP。

关节角定义（全部为右手定则，0 = CAD 装配姿态 = 零点工装姿态）
  th1 (HAA)  绕 +x：  > 0 时腿向左外摆（足端向 +y 抬起侧）
  th2 (HFE)  绕 +y：  > 0 时大腿向后摆（Ry 正向把 +x 转向 -z、把 -z 转向 -x）
  th3 (膝)   小腿相对大腿绕 +y：> 0 时小腿向下伸直；th3 = +90° 为腿完全伸直
零位姿态：大腿竖直向下，小腿水平朝前，足端球心在 (210, 120.5, -213)。
"""
from dataclasses import dataclass, field
import numpy as np


# ====================================================================== 参数
@dataclass
class LegGeometry:
    d: float = 120.5          # 腿平面到 HAA 轴的侧向偏距（足端球心 / 连杆中面）
    L1: float = 213.0         # 大腿：HFE 轴 -> 膝铰链销
    L2: float = 210.0         # 小腿：膝铰链销 -> 足端球心
    foot_r: float = 25.71     # 足端球半径（触地点 = 球心沿地面法向偏移 foot_r）
    z_pinion: int = 18        # 膝电机输出齿轮齿数
    z_gear: int = 26          # 大腿上的从动齿轮齿数
    # 平行四边形死点：曲柄与机架共线时（零位时二者夹角 99.06°）
    knee_change_lo: float = np.radians(-99.06)   # 小腿向大腿折叠方向的死点
    knee_change_hi: float = np.radians(+80.94)   # 伸直方向的死点（th3=90° 为完全伸直）

    @property
    def knee_ratio(self):
        """th3 = knee_ratio * (膝电机相对大腿的转角)。外啮合，反向。"""
        return -self.z_pinion / self.z_gear      # = -0.6923


@dataclass
class Calibration:
    """关节角 -> 驱动器指令角：
         q1 = s1*th1 + o1
         q2 = s2*th2 + o2
         q3 = s3*(th3/knee_ratio) + o3        （膝电机相对大腿转 th3/knee_ratio）
       sign: 驱动器正方向与模型正方向（HAA 绕 +x，HFE/膝绕 +y）一致为 +1，否则 -1
       offset: 装上零点工装（零位姿态）时驱动器的读数（rad）
       注意：膝电机定子装在大腿上，所以 q3 本身就是相对大腿的角度，不需要再减去 q2。"""
    sign: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))


def Rx(t):
    c, s = np.cos(t), np.sin(t); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(t):
    c, s = np.cos(t), np.sin(t); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

# CAD -> 髋坐标系：p_hip = CAD2HIP @ (p_cad - H_CAD)
H_CAD = np.array([-270.9, 238.0, 0.0])
CAD2HIP = np.array([[-1, 0, 0],
                    [0, 0, 1],
                    [0, 1, 0]], float)


# ====================================================================== 模型
class LegModel:
    def __init__(self, geometry: LegGeometry = None, calib: Calibration = None, knee_margin_deg=10.0):
        self.g = geometry or LegGeometry()
        self.cal = calib or Calibration()
        self.knee_margin = np.radians(knee_margin_deg)   # 离平行四边形死点至少留的余量

    # ---------------- 关节空间
    def fk(self, th):
        """关节角 (th1, th2, th3) -> 足端球心位置（髋坐标系）。"""
        th1, th2, th3 = th; g = self.g
        thigh = np.array([0.0, 0.0, -g.L1])
        shank = Ry(th3) @ np.array([g.L2, 0.0, 0.0])
        return Rx(th1) @ (np.array([0.0, g.d, 0.0]) + Ry(th2) @ (thigh + shank))

    def ik(self, p):
        """足端球心位置 -> 关节角。返回 (th, ok, reason)。
        取"腿在髋下方、膝角 |th3| < 90°"的那一支解。"""
        px, py, pz = p; g = self.g
        rr = py**2 + pz**2 - g.d**2
        if rr < 0:
            return np.full(3, np.nan), False, "足端离 HAA 轴太近（小于侧偏 d）"
        zp = -np.sqrt(rr)                                   # 腿平面内的竖直分量（向下）
        th1 = np.arctan2(pz, py) - np.arctan2(zp, g.d)
        s = (px**2 + zp**2 - g.L1**2 - g.L2**2) / (2 * g.L1 * g.L2)
        if abs(s) > 1:
            return np.full(3, np.nan), False, "超出腿长（太远或太近）"
        th3 = np.arcsin(s)
        vx, vz = g.L2 * np.cos(th3), -g.L1 - g.L2 * np.sin(th3)
        th2 = np.arctan2(vz, vx) - np.arctan2(zp, px)
        th = np.array([th1, (th2 + np.pi) % (2 * np.pi) - np.pi, th3])
        ok = (g.knee_change_lo + self.knee_margin) <= th3 <= (g.knee_change_hi - self.knee_margin)
        return th, bool(ok), "" if ok else "膝角接近平行四边形死点"

    def jacobian(self, th, h=1e-7):
        J = np.zeros((3, 3))
        for k in range(3):
            e = np.zeros(3); e[k] = h
            J[:, k] = (self.fk(th + e) - self.fk(th - e)) / (2 * h)
        return J

    # ---------------- 电机空间
    def joint_to_motor(self, th):
        c, g = self.cal, self.g
        return np.array([c.sign[0] * th[0] + c.offset[0],
                         c.sign[1] * th[1] + c.offset[1],
                         c.sign[2] * th[2] / g.knee_ratio + c.offset[2]])

    def motor_to_joint(self, q):
        c, g = self.cal, self.g
        return np.array([(q[0] - c.offset[0]) / c.sign[0],
                         (q[1] - c.offset[1]) / c.sign[1],
                         (q[2] - c.offset[2]) / c.sign[2] * g.knee_ratio])

    def foot_to_motor(self, p):
        """p: (3,) 或 (N,3) 足端球心位置(mm, 髋坐标系) -> 电机指令角(rad)。
        info['ok']=False 的点不要下发。"""
        arr = np.atleast_2d(np.asarray(p, float)); out = []; infos = []
        for row in arr:
            th, ok, why = self.ik(row)
            out.append(self.joint_to_motor(th)); infos.append(dict(joint=th, ok=ok, reason=why))
        out = np.array(out)
        return (out[0], infos[0]) if np.ndim(p) == 1 else (out, infos)

    def motor_to_foot(self, q):
        arr = np.atleast_2d(np.asarray(q, float))
        out = np.array([self.fk(self.motor_to_joint(row)) for row in arr])
        return out[0] if np.ndim(q) == 1 else out

    def motor_jacobian(self, q):
        """足端速度 = Jm @ 电机角速度。静力学：电机力矩 = Jm^T @ 足端力。"""
        th = self.motor_to_joint(q)
        D = np.diag([1 / self.cal.sign[0], 1 / self.cal.sign[1], self.g.knee_ratio / self.cal.sign[2]])
        return self.jacobian(th) @ D


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    leg = LegModel(); d = np.degrees
    print("零位足端（髋坐标系）:", leg.fk(np.zeros(3)), " CAD 足端球心换算:",
          CAD2HIP @ (np.array([-480.9, 25.0, 120.5]) - H_CAD))
    print("膝传动比 th3/q3 =", round(leg.g.knee_ratio, 4))
    tests = [[0, 0, 0], [10, 20, 30], [-15, -30, 45], [5, 40, -40], [0, -10, 70]]
    print("  关节角(deg)          ->  足端(mm)               -> IK 往返误差(deg)  电机角(deg)")
    for t in tests:
        th = np.radians(t); p = leg.fk(th); q, info = leg.foot_to_motor(p)
        print(f"  {str(t):20s} {p} {np.max(np.abs(d(info['joint'] - th))):.1e}  {d(q)}  ok={info['ok']}")
    # 站立姿态示例：足端在髋正下方 300 mm
    q, info = leg.foot_to_motor([0, 120.5, -300])
    print("足端 (0,120.5,-300):", "关节角", d(info["joint"]), "电机角", d(q), info["ok"])
    print("该姿态 J（足端 mm / 关节 rad）:\n", leg.jacobian(info["joint"]))
