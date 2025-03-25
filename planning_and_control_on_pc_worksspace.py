#!/usr/bin/env python3
import serial
import numpy as np

# -------------------------------
# 全局变量：机器人状态
# -------------------------------
# 当前位姿 (x, y, z, roll, pitch, yaw)
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
    # 请根据实际模型实现
    target_lengths = some_inverse_kinematics_function(target_pose)
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

def quintic_trajectory(s0, sT, v0, vT, a0, aT ,T, num_samples):
    """
    利用五次多项式规划单个标量轨迹（初始和终点速度、加速度均为 0）
    返回一个包含 num_samples 个采样点的列表
    """
    # 初始速度、加速度均为0
    #v0 = 0; vT = 0; a0 = 0; aT = 0;
    a0_coef = s0
    a1_coef = v0
    a2_coef = a0 / 2.0
    a3_coef = (20*(sT-s0) - (8*vT + 12*v0)*T - (3*a0 - aT)*T**2) / (2 * T**3)
    a4_coef = (30*(s0-sT) + (14*vT + 16*v0)*T + (3*a0 - 2*aT)*T**2) / (2 * T**4)
    a5_coef = (12*(sT-s0) - (6*vT + 6*v0)*T - (a0 - aT)*T**2) / (2 * T**5)
    
    t_vals = np.linspace(0, T, num_samples)
    s_vals = a0_coef + a1_coef * t_vals + a2_coef * t_vals**2 + a3_coef * t_vals**3 + a4_coef * t_vals**4 + a5_coef * t_vals**5
    return s_vals.tolist()

def workspace_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples):
    """
    对平台的位姿进行五次多项式轨迹规划
    其中 start_pose 和 target_pose 为6维向量，返回一个包含 num_samples 个6维轨迹点的列表
    假设初始与终点的速度和加速度均为 0
    """
    traj = []
    trajectories = []
    for i in range(6):
        traj_i = quintic_trajectory(start_pose[i], target_pose[i], start_velocity[i], target_velocity[i], start_acceleration[i], target_acceleration[i], 
                                    T, num_samples)
        trajectories.append(traj_i)
    # 组合各自由度的轨迹点
    for j in range(num_samples):
        point = [trajectories[i][j] for i in range(6)]
        traj.append(point)
    return traj

def move_to_target(target_lengths):
    """
    直接移动到目标绳长：计算步进量后发送指令，
    并更新全局变量 current_lengths
    """
    global current_lengths
    steps = compute_motor_steps(current_lengths, target_lengths)
    send_motor_steps(steps)
    current_lengths = target_lengths

def execute_trajectory_workspace(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples):
    """
    在工作空间进行轨迹规划：
      1. 首先将机器人移动到起始位姿 start_pose（先求解 IK 得到初始绳长，再移动）
      2. 然后利用五次多项式对平台位姿进行规划，
         对轨迹上每个位姿点，求解逆解，得到对应的绳长，
         依次发送指令，完成整段运动。
    """
    global current_pose, current_lengths
    
    # 移动到起始位姿
    s_start = inverse_kinematics(start_pose)
    move_to_target(s_start)
    current_pose = start_pose.copy()  # 更新当前位姿
    print("\n当前位姿:", current_pose)
    print("当前绳长:", current_lengths)
    
    # 规划工作空间轨迹（轨迹上的每个点为 6 维位姿）
    pose_traj = workspace_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples)
    
    # 对轨迹上每个位姿点，求逆解得到目标绳长，并运动
    for pose in pose_traj:
        s_target = inverse_kinematics(pose)
        move_to_target(s_target)
        current_pose = pose  # 更新当前位姿
        print("\n当前位姿:", current_pose)
        print("当前绳长:", current_lengths)

def manual_control(key_input):
    """
    手动控制模式：根据键盘输入调整 6 自由度位姿，
    然后计算新的目标绳长并移动到该位姿
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

    s_target = inverse_kinematics(current_pose)
    move_to_target(s_target)
    print("\n当前位姿:", current_pose)
    print("当前绳长:", current_lengths)

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
            # 轨迹规划模式示例：在工作空间规划轨迹
            # 示例起始位姿和目标位姿（6维向量）
            start_pose = [0.5, 0.5, 0.3, 0, 0, 0]
            target_pose = [0.6, 0.6, 0.35, 0.05, 0.05, 0.1]
            # T = 2秒，50个采样点
            execute_trajectory_workspace(start_pose, target_pose,[0]*6,[0]*6,[0]*6,[0]*6, 2, 50)
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
