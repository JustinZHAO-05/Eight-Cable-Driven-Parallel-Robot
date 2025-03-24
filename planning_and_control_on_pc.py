#!/usr/bin/env python3
import serial
import numpy as np

# -------------------------------
# 全局变量：机器人状态
# -------------------------------
# 当前位姿 (x, y, z, alpha, beta, gamma)
current_pose = [0, 0, 0, 0, 0, 0]  
# 当前绳长（8 根绳子）
current_lengths = [0] * 8  

# -------------------------------
# 函数定义
# -------------------------------

def inverse_kinematics(target_pose):
    """
    计算逆解，返回目标绳长列表
    此处使用你自己的机器人模型，替换 some_inverse_kinematics_function
    """
    target_lengths = some_inverse_kinematics_function(target_pose)  # 请自行实现
    return target_lengths

def compute_motor_steps(current_lengths, target_lengths, step_angle=1.8, reduction_ratio=1, drum_radius=0.02):
    """
    根据当前绳长和目标绳长计算电机需要移动的步数（取整）
    """
    steps = []
    for i in range(8):
        delta_s = target_lengths[i] - current_lengths[i]  # 绳长变化
        steps_per_revolution = 360 / step_angle
        length_per_rev = 2 * np.pi * drum_radius
        delta_steps = (delta_s / length_per_rev) * steps_per_revolution * reduction_ratio
        steps.append(int(delta_steps))
    return steps

# 初始化串口：请根据实际情况修改 COM 端口和波特率
ser = serial.Serial('COM3', 115200)

def send_motor_steps(steps):
    """
    将步数指令转换为字符串并通过串口发送给 Arduino
    格式示例："M 500 -300 200 0 100 0 -150 250\n"
    """
    command = "M " + " ".join(map(str, steps)) + "\n"
    ser.write(command.encode())

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



    # 生成时间采样点
    t_vals = np.linspace(0, T, num_samples)
    # 根据多项式计算轨迹点
    s_vals = a0 + a1 * t_vals + a2 * t_vals**2 + a3 * t_vals**3 + a4 * t_vals**4 + a5 * t_vals**5
    return s_vals.tolist()

def compute_full_trajectory(s_init, s_target, v_init, v_target, a_init, a_target, T, num_samples):
    """
    对 8 根绳子分别计算轨迹，并返回每个时刻的 8 维目标绳长组成的列表
    """
    trajectory = []
    for i in range(8):
        traj = trajectory_planning(s_init[i], s_target[i], v_init[i], v_target[i], a_init[i], a_target[i], T, num_samples)
        trajectory.append(traj)
    # 组合为每个时间点对应 8 根绳子的目标值
    return list(zip(*trajectory))

def move_to_target(target_lengths):
    """
    直接移动到目标绳长：计算步进量后发送指令，
    并更新全局变量 current_lengths
    """
    global current_lengths
    steps = compute_motor_steps(current_lengths, target_lengths)
    send_motor_steps(steps)
    current_lengths = target_lengths

def execute_trajectory(s_init, s_target, v_init, v_target, a_init, a_target, T, num_samples):
    """
    先将机器人移动到起始位置 s_init，
    然后按照规划的轨迹逐步移动到 s_target
    """
    global current_lengths
    move_to_target(s_init)  # 先归位到起始位置
    trajectory = compute_full_trajectory(s_init, s_target, v_init, v_target, a_init, a_target, T, num_samples)
    for step in trajectory:
        steps = compute_motor_steps(current_lengths, step)
        send_motor_steps(steps)
        current_lengths = list(step)  # 更新当前绳长

def manual_control(key_input):
    """
    手动控制模式：根据键盘输入调整 6 自由度位姿，
    然后计算新的目标绳长并移动到该位置
    """
    global current_pose
    delta = 0.01  # 位姿增量
    if key_input == 'w': current_pose[0] += delta
    elif key_input == 's': current_pose[0] -= delta
    elif key_input == 'a': current_pose[1] += delta
    elif key_input == 'd': current_pose[1] -= delta
    elif key_input == 'q': current_pose[2] += delta
    elif key_input == 'e': current_pose[2] -= delta
    elif key_input == 'i': current_pose[3] += delta
    elif key_input == 'k': current_pose[3] -= delta
    elif key_input == 'j': current_pose[4] += delta
    elif key_input == 'l': current_pose[4] -= delta
    elif key_input == 'u': current_pose[5] += delta
    elif key_input == 'o': current_pose[5] -= delta

    target_lengths = inverse_kinematics(current_pose)
    move_to_target(target_lengths)

def initialize_robot():
    """
    初始化：要求手动将机器人移动到世界坐标系原点，
    然后发送 "TIGHTEN" 指令给 Arduino 以拉紧绳索，
    等待 Arduino 返回确认信息
    """
    input("请手动将机器人移动到世界坐标系原点，并按回车继续...")
    ser.write(b"TIGHTEN\n")
    response = ser.readline().decode().strip()
    if response == "TIGHTEN_OK":
        print("绳索已拉紧，初始化完成！")
    else:
        print("拉紧失败，请检查连接！")

def main():
    initialize_robot()

    while True:
        print("\n当前位姿:", current_pose)
        print("当前绳长:", current_lengths)
        mode = input("请选择模式（1=轨迹规划，2=手动控制，q=退出）：")
        if mode == "1":
            # 轨迹规划模式示例：这里使用示例位姿
            s_target = inverse_kinematics([0.5, 0.5, 0.3, 0, 0, 0])
            # 这里以 0 作为初始速度、0 作为目标速度与加速度；T = 2 秒，50 个采样点
            execute_trajectory(s_target, [0]*8, [0]*8, [0]*8, [0]*8, [0]*8, 2, 50)
        elif mode == "2":
            # 手动控制模式
            while True:
                key_input = input("控制（w/s/a/d/q/e/i/k/j/l/u/o），输入 b 返回：")
                if key_input == "b":
                    break
                manual_control(key_input)
        elif mode == "q":
            break

if __name__ == "__main__":
    main()
