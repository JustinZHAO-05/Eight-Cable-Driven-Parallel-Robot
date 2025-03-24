import numpy as np

def trajectory_planning(s0, sT, v0, vT, acc0, accT, T, num_samples):
    """
    利用五次多项式规划单根绳长的运动轨迹

    参数：
        s0      : 初始位置
        sT      : 终点位置
        v0      : 初始速度
        vT      : 终点速度
        acc0    : 初始加速度
        accT    : 终点加速度
        T       : 总运动时间（秒）
        num_samples : 采样点数量

    返回：
        一个包含 num_samples 个轨迹采样点的列表
    """
    # 直接确定前三个系数
    a0 = s0
    a1 = v0
    a2 = acc0 / 2.0

    # 计算剩余三个系数
    a3 = (20*(sT - s0) - (8*vT + 12*v0)*T - (3*acc0 - accT)*T**2) / (2 * T**3)
    a4 = (30*(s0 - sT) + (14*vT + 16*v0)*T + (3*acc0 - 2*accT)*T**2) / (2 * T**4)
    a5 = (12*(sT - s0) - (6*vT + 6*v0)*T - (acc0 - accT)*T**2) / (2 * T**5)

    # 打印系数以便调试（可选）
    print("Polynomial Coefficients:")
    print(f"a0 = {a0}, a1 = {a1}, a2 = {a2}, a3 = {a3}, a4 = {a4}, a5 = {a5}")

    # 生成时间采样点
    t_vals = np.linspace(0, T, num_samples)
    # 根据多项式计算轨迹点
    s_vals = a0 + a1 * t_vals + a2 * t_vals**2 + a3 * t_vals**3 + a4 * t_vals**4 + a5 * t_vals**5
    return s_vals.tolist()

# 示例使用
if __name__ == "__main__":
    # 示例初始和目标条件
    s0 = 0.0         # 初始位置
    sT = 100.0       # 终点位置
    v0 = 0.0         # 初始速度
    vT = 0.0         # 终点速度
    acc0 = 0.0       # 初始加速度
    accT = 0.0       # 终点加速度
    T = 5.0          # 总运动时间 5 秒
    num_samples = 50 # 50 个采样点

    traj = trajectory_planning(s0, sT, v0, vT, acc0, accT, T, num_samples)
    print("Trajectory Points:")
    print(traj)
