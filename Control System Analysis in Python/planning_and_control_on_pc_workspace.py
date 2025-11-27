#!/usr/bin/env python3
import serial
import numpy as np
import time
import math
from no_longer_use.IK_calculation import calculate_cable_lengths 
# -------------------------------
# Global Variables: Robot State
# -------------------------------
# current_pose: [x, y, z, roll, pitch, yaw]
current_pose = [34, 28, -5, 0, 0, 0] #cm
# current_lengths: cable lengths for 8 cables
current_lengths = [84.79,84.79,84.79,84.79,38.01,38.01,38.01,38.01]  

# ------------------------------- 
# Function Definitions
# -------------------------------

def inverse_kinematics(target_pose):
    """
    Compute the inverse kinematics to obtain the target cable lengths.
    Calls the calculate_cable_lengths() function (imported from IK_calculation).
    """
    target_lengths = calculate_cable_lengths(target_pose)
    return target_lengths

def compute_motor_degrees_speed(current_lengths, target_lengths, sample_time, reduction_ratio=1, drum_radius=0.4): #cm
    """
    Compute the number of steps required for each motor based on the change in cable length.
    Returns a list of 8 integer step values.
    """
    
    commands = []

    for i in range(8):
        CONTROL_ID = i 
        delta_s = target_lengths[i] - current_lengths[i]  # Change in cable length
        length_per_rev = 2 * np.pi * drum_radius
        delta_degrees = (delta_s / length_per_rev) * 360 * reduction_ratio
        if delta_degrees >= 0:
            pos = int(delta_degrees * 10)
            degrees_h=(pos >> 8) & 0xFF
            degrees_l=pos & 0xFF
            if i==3 or i ==5 or i == 7:
                DIR = 0x01
            else:
                DIR = 0x00
            spd = int((math.radians(delta_degrees)/sample_time) * 10)
            speeds_h=(spd >> 8) & 0xFF
            speeds_l=spd & 0xFF
            frame = [
            HEADER,
            CONTROL_ID,
            MODE,
            DIR,
            MICROSTEP,
            degrees_h, degrees_l,
            speeds_h, speeds_l
            ]
            bcc = 0
            for b in frame:
                bcc ^= b
            # 完整帧：9 字节 + BCC + 帧尾
            frame.append(bcc)
            frame.append(0x7D)  # 帧尾
            commands.append(bytes(frame))

        else:
            pos = int(-delta_degrees * 10)
            degrees_h=(pos >> 8) & 0xFF
            degrees_l=pos & 0xFF
            if i == 3 or i == 5 or i == 7 :
                DIR = 0x00
            else:
                DIR= 0x01
            spd = int((math.radians(-delta_degrees)/sample_time) * 10)
            speeds_h=(spd >> 8) & 0xFF
            speeds_l=spd & 0xFF
            frame = [
            HEADER,
            CONTROL_ID,
            MODE,
            DIR,
            MICROSTEP,
            degrees_h, degrees_l,
            speeds_h, speeds_l
            ]
            bcc = 0
            for b in frame:
                bcc ^= b
            # 完整帧：9 字节 + BCC + 帧尾
            frame.append(bcc)
            frame.append(TAIL)  # 帧尾
            commands.append(bytes(frame))


    return commands

# Initialize serial port; adjust COM port and baud rate as needed
ser0 = serial.Serial('COM11', 115200)
ser1 = serial.Serial('COM16', 115200)
ser2 = serial.Serial('COM13', 115200)
ser3 = serial.Serial('COM10', 115200)
ser4 = serial.Serial('COM14', 115200)
ser5 = serial.Serial('COM9', 115200)
ser6 = serial.Serial('COM12', 115200)
ser7 = serial.Serial('COM15', 115200)

SER = [ser0,ser1,ser2,ser3,ser4,ser5,ser6,ser7]

HEADER     = 0x7B   # 帧头
MODE       = 0x02   # 0x02 = 位置控制模式
MICROSTEP  = 0x20   # 0x20 = 32细分
TAIL      = 0x7D

def send_motor(commands):
    """
    Convert the step command to a string and send it over serial to Arduino.
    The command string now includes the sample_time information.
    Example format: "M 500 -300 200 0 100 0 -150 250;T:80\n"
    where 80 (ms) is the time interval between samples.
    
    After sending the actual command, an empty command (just a newline)
    is sent immediately. This empty command helps ensure that Arduino clears
    out the previous command from its buffer and avoids repeatedly executing
    the same movement command.
    """
    # Construct the command string with the sample time information.
    for i in range(8):
        print(commands[i])
        SER[i].write(commands[i])
    
    return





def quintic_trajectory(s0, sT, v0, vT, a0, aT, T, num_samples):
    """
    Plan a scalar trajectory using a quintic polynomial given the boundary conditions.
    Returns a list of num_samples trajectory points.
    """
    a0_coef = s0
    a1_coef = v0
    a2_coef = a0 / 2.0
    a3_coef = (20*(sT - s0) - (8*vT + 12*v0)*T - (3*a0 - aT)*T**2) / (2 * T**3)
    a4_coef = (30*(s0 - sT) + (14*vT + 16*v0)*T + (3*a0 - 2*aT)*T**2) / (2 * T**4)
    a5_coef = (12*(sT - s0) - (6*vT + 6*v0)*T - (a0 - aT)*T**2) / (2 * T**5)
    
    t_vals = np.linspace(0, T, num_samples)
    s_vals = a0_coef + a1_coef * t_vals + a2_coef * t_vals**2 + a3_coef * t_vals**3 + a4_coef * t_vals**4 + a5_coef * t_vals**5
    return s_vals.tolist()

def workspace_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples):
    """
    Plan a trajectory in workspace (pose) space using quintic polynomials.
    Both start_pose and target_pose are 6-element vectors.
    Returns a list of num_samples pose points (each a 6-element list).
    """
    trajectories = []
    for i in range(6):
        traj_i = quintic_trajectory(start_pose[i], target_pose[i], start_velocity[i],
                                    target_velocity[i], start_acceleration[i], target_acceleration[i],
                                    T, num_samples)
        trajectories.append(traj_i)
    # Combine into a list of pose points
    traj = []
    for j in range(num_samples):
        point = [trajectories[i][j] for i in range(6)]
        traj.append(point)
    return traj

def move_to_target(target_lengths, sample_time):
    """
    Move the robot to the target cable lengths by
    1) 计算每路命令帧
    2) 发送给 8 台电机
    3) 等待每台电机在位置控制模式下的“到位”反馈
    4) 更新 current_lengths

    target_lengths: list of 8 floats, 目标绳长 (cm)
    sample_time    : 间隔时间 (ms)，用于计算帧内速度字段
    """
    global current_lengths

    # 1) 计算所有电机的命令帧
    commands = compute_motor_degrees_speed(current_lengths,
                                           target_lengths,
                                           (sample_time-15)/1000.0)

    # 2) 并行写入 8 个串口
    send_motor(commands)

    print("write done")

    #3) 握手确认：等待每台电机发 9 字节反馈，且反馈第二字节=0x01（表示到达目标）
    # reached = [False] * 8
    # while not all(reached):
    #     for idx, ser in enumerate(SER):
    #         if idx == 0: 
    #             reached[idx] = 1
    #         # 如果有至少 9 字节待读
    #         elif ser.in_waiting >= 9:
    #             data = ser.read(9)
    #             # 校验帧长度
    #             if len(data) != 9:
    #                 continue
    #             addr = data[0]      # 电机地址 0…7
    #             status = data[1]    # 在位置控制下，到位=0x01，未到=0x00
    #             # 只标记本地址为到位
    #             if status == 0x01 and 0 <= addr <= 7:
    #                 reached[addr] = True
    #     # 非阻塞短暂休眠，防止 busy‐wait
    #     time.sleep(0.001)
    time.sleep((sample_time)/1000.0)

    # 4) 全部到位后，更新状态
    current_lengths = target_lengths.copy()



def execute_trajectory_workspace(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples):
    """
    Plan and execute a workspace trajectory:
      1. Move to the starting pose (via inverse kinematics).
      2. Generate a trajectory in the workspace, and for each trajectory point, compute the 
         corresponding cable lengths (IK) and send commands.
         Each command includes the sample time between trajectory points.
    """
    global current_pose, current_lengths
    
    start_pose = convert_angles_deg_to_rad(start_pose)
    target_pose = convert_angles_deg_to_rad(target_pose)
    start_velocity = convert_angles_deg_to_rad(start_velocity)
    target_velocity = convert_angles_deg_to_rad(target_velocity)
    start_acceleration = convert_angles_deg_to_rad(start_acceleration)
    target_acceleration = convert_angles_deg_to_rad(target_acceleration)

    length0 = []
    for i in range(6):
        length0.append((start_pose[i]-current_pose[i])**2)
    T0 = int(float(np.sqrt(length0[0]+length0[1]+length0[2]+length0[3]+length0[4]+length0[5]))/3*1000)

    print('T0=',T0/1000,"s")

    # Move to the starting pose
    s_start = inverse_kinematics(start_pose)
    # Compute sample time (in ms) between trajectory points
    sample_time = int((T * 1000) / num_samples)
    move_to_target(s_start, T0)
    current_pose = start_pose.copy()
    print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
    print("Current cable lengths:", current_lengths)
    
    # Generate workspace trajectory (each point is a 6D pose)
    pose_traj = workspace_trajectory_planning(start_pose, target_pose, start_velocity,
                                              target_velocity, start_acceleration, target_acceleration,
                                              T, num_samples)
    # For each pose point, compute IK and move
    con_traj = []

    for pose in pose_traj:
        con_traj.append(inverse_kinematics(pose))

    input("start now?")

    for i in range(num_samples):
        move_to_target(con_traj[i], sample_time)
        current_pose = pose_traj[i]  # Update current pose
        print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
        print("Current cable lengths:", current_lengths)


def trajectory_workspace_serial(start_pose, target_pose, start_velocity, target_velocity,
                                  start_acceleration, target_acceleration, T, num_samples):
    """
    Plan and execute a workspace trajectory:
      1. Move to the starting pose (via inverse kinematics).
      2. Generate a trajectory in the workspace, and for each trajectory point, compute the 
         corresponding cable lengths (IK) and send commands.
         Each command includes the sample time between trajectory points.
    """
    global current_pose, current_lengths
    
    start_pose = convert_angles_deg_to_rad(start_pose)
    target_pose = convert_angles_deg_to_rad(target_pose)
    start_velocity = convert_angles_deg_to_rad(start_velocity)
    target_velocity = convert_angles_deg_to_rad(target_velocity)
    start_acceleration = convert_angles_deg_to_rad(start_acceleration)
    target_acceleration = convert_angles_deg_to_rad(target_acceleration)

    print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
    print("Current cable lengths:", current_lengths)
    
    # Generate workspace trajectory (each point is a 6D pose)
    pose_traj = workspace_trajectory_planning(start_pose, target_pose, start_velocity,
                                              target_velocity, start_acceleration, target_acceleration,
                                              T, num_samples)
    # For each pose point, compute IK and move
    con_traj = []

    for pose in pose_traj:
        con_traj.append(inverse_kinematics(pose))
    sample_time = int((T * 1000) / num_samples)
    return pose_traj,con_traj,sample_time,num_samples



def execute_traj_serial(pose_traj_serial,trajs_serial,sample_time,num_samples):

    global current_lengths,current_pose

    for j in range(len(trajs_serial)):
        for i in range(num_samples[j]):
            move_to_target(trajs_serial[j][i], sample_time[j])
            current_pose = pose_traj_serial[j][i]  # Update current pose
            print(current_pose)
            print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
            print("Current cable lengths:", current_lengths)

def manual_control(key_input):
    """
    Manual control mode: Adjust the current pose based on keyboard input,
    then compute the new target cable lengths and move to that pose.
    """
    global current_pose
    global current_lengths
    delta = 1  # Pose increment
    if key_input == 'w': current_pose[0] += delta
    elif key_input == 's': current_pose[0] -= delta
    elif key_input == 'a': current_pose[1] += delta
    elif key_input == 'd': current_pose[1] -= delta
    elif key_input == 'q': current_pose[2] += 3*delta
    elif key_input == 'e': current_pose[2] -= 3*delta
    elif key_input == 'i': current_pose[3] += delta*3.1415926/180
    elif key_input == 'k': current_pose[3] -= delta*3.1415926/180
    elif key_input == 'j': current_pose[4] += delta*3.1415926/180
    elif key_input == 'l': current_pose[4] -= delta*3.1415926/180
    elif key_input == 'u': current_pose[5] += delta*3.1415926/180
    elif key_input == 'o': current_pose[5] -= delta*3.1415926/180

    s_target = inverse_kinematics(current_pose)
    # For manual control, use a default sample time (e.g., 50 ms)
    move_to_target(s_target, 50)
    print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
    print("Current cable lengths:", current_lengths)



# 串口列表，对应电机 1~8
#COMS = ['COM8','COM9','COM10','COM11','COM12','COM13','COM14','COM15']
#sers = [serial.Serial(port, 115200, timeout=0.1) for port in COMS]

# 协议常量
HEADER    = 0x7B
MODE_SPEED= 0x01
DIR_CW    = 0x01
MICROSTEP = 0x20
TAIL      = 0x7D

# 速度字段：10 rad/s -> 100 -> 高低字节
speed_val = 10 * 10       # rad/s ×10
SPEED_H = (speed_val >> 8) & 0xFF
SPEED_L = speed_val &  0xFF

def build_frame(motor_id, speed_h, speed_l):
    """
    构造一次速度控制帧（速度模式，POS=0，方向=CW）。
    """
    if motor_id == 3 or motor_id == 5 or motor_id == 7 :
        DIR_CW_i = 0x00
    else :
        DIR_CW_i = 0x01
    frame = [
        HEADER,
        motor_id,
        MODE_SPEED,
        DIR_CW_i,
        MICROSTEP,
        0x00, 0x00,        # POS_H, POS_L
        speed_h, speed_l   # SPEED_H, SPEED_L
    ]
    # 计算 BCC
    bcc = 0
    for b in frame:
        bcc ^= b
    frame.append(bcc)
    frame.append(TAIL)
    print(bytes(frame))
    return bytes(frame)

def init_cables():
    for idx, ser in enumerate(SER, start=0):
        # 构造“启动”帧
        input("启动下一个")
        print("idx=",idx,"ser=",ser)
        start_frame = build_frame(idx, SPEED_H, SPEED_L)
        ser.write(start_frame)
        print(f"⚙️  Motor {idx} running @10 rad/s CW.  → 按 Enter 停止")
        
        input()  # 等待用户回车
        
        # 构造“停止”帧（速度 = 0）
        stop_frame = build_frame(idx, 0x00, 0x00)
        ser.write(stop_frame)
        print(f"⏹  Motor {idx} stopped.")
        
        # 给一点缓冲时间，确保命令被处理
        time.sleep(0.1)




def initialize_robot():
    """
    Initialize the robot by requiring manual placement to the origin,
    then send a "TIGHTEN" command to pre-tension the cables,
    and wait for the Arduino to respond with confirmation.
    """
    input("Please manually move the robot to the origin and press Enter to continue...")
    print(">>> 从 1 到 8 号电机依次初始化：10 rad/s CW。")
    init_cables()
    print("✅ 全部电机初始化完毕。")





def convert_angles_deg_to_rad(lst):
    """
    Given a 6-element list, convert the last three elements (angles in degrees)
    to radians, and return the new list.
    
    Parameters:
        lst (list): A list with 6 numerical elements, where the last three are angles in degrees.
    
    Returns:
        list: A new list with the first three elements unchanged and the last three elements converted to radians.
    """
    if len(lst) != 6:
        raise ValueError("Input list must have exactly 6 elements.")
    # Copy the first three elements (position values)
    result = lst[:3]
    # Convert the last three elements (angles) from degrees to radians
    for angle in lst[3:]:
        result.append(math.radians(angle))
    return result

def convert_angles_rad_to_deg(lst):
    """
    Given a 6-element list, convert the last three elements (angles in radians)
    to degrees, and return the new list.
    
    Parameters:
        lst (list): A list with 6 numerical elements, where the last three are angles in radians.
    
    Returns:
        list: A new list with the first three elements unchanged and the last three elements converted to degrees.
    """
    if len(lst) != 6:
        raise ValueError("Input list must have exactly 6 elements.")
    # Copy the first three elements (position values)
    result = lst[:3]
    # Convert the last three elements (angles) from radians to degrees
    for angle in lst[3:]:
        result.append(math.degrees(angle))
    return result

def main():
    initialize_robot()

    while True:
        print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
        print("Current cable lengths:", current_lengths)
        mode = input("Choose mode (1=Trajectory Planning, 2=Manual Control, q=Quit): ")
        if mode == "1":
            # Trajectory planning mode:
            # Example starting and target poses (6D vectors)
            start_pose = [34, 28, 33, 0, 0, 0]
            target_pose = [38, 38, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity = [0]*6
            target_velocity = [0]*6
            start_acceleration = [0]*6
            target_acceleration = [0]*6
            # T is the total motion time (seconds), num_samples is the number of trajectory points

            start_pose_2 = target_pose
            target_pose_2 = [25, 38, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_2 = target_velocity
            target_velocity_2 = [0]*6
            start_acceleration_2 = target_acceleration
            target_acceleration_2 = [0]*6

            pose2,traj2,sample_time2,num_sample2=trajectory_workspace_serial(start_pose_2, target_pose_2, start_velocity_2, target_velocity_2,
                                         start_acceleration_2, target_acceleration_2, 7, 100)
            
            start_pose_3 = target_pose_2
            target_pose_3 = [25, 25, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_3 = target_velocity_2
            target_velocity_3 = [0]*6
            start_acceleration_3 = target_acceleration_2
            target_acceleration_3 = [0]*6

            pose3,traj3,sample_time3,num_sample3=trajectory_workspace_serial(start_pose_3, target_pose_3, start_velocity_3, target_velocity_3,
                                         start_acceleration_3, target_acceleration_3, 7, 100)
            
            start_pose_4 = target_pose_3
            target_pose_4 = [38, 25, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_4 = target_velocity_3
            target_velocity_4 = [0]*6
            start_acceleration_4 = target_acceleration_3
            target_acceleration_4 = [0]*6

            pose4,traj4,sample_time4,num_sample4=trajectory_workspace_serial(start_pose_4, target_pose_4, start_velocity_4, target_velocity_4,
                                         start_acceleration_4, target_acceleration_4, 7, 100)
            
            start_pose_5 = target_pose_4
            target_pose_5 = [38, 38, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_5 = target_velocity_4
            target_velocity_5 = [0]*6
            start_acceleration_5 = target_acceleration_4
            target_acceleration_5 = [0]*6

            pose5,traj5,sample_time5,num_sample5=trajectory_workspace_serial(start_pose_5, target_pose_5, start_velocity_5, target_velocity_5,
                                         start_acceleration_5, target_acceleration_5, 7, 100)
            
            
            
            poses=[pose2,pose3,pose4,pose5]
            
            trajs =[traj2,traj3,traj4,traj5]
            sample_times = [sample_time2,sample_time3,sample_time4,sample_time5]
            num_samples = [num_sample2,num_sample3,num_sample4,num_sample5]

            execute_trajectory_workspace(start_pose, target_pose, start_velocity, target_velocity,
                                         start_acceleration, target_acceleration, 7, 100)
            
            execute_traj_serial(poses,trajs,sample_times,num_samples)
            
            

            
            
        elif mode == "2":
            # Manual control mode
            while True:
                key_input = input("Control (w/s/a/d/q/e/i/k/j/l/u/o), type 'b' to go back: ")
                if key_input == "b":
                    break
                manual_control(key_input)
        elif mode == "q":
            break

if __name__ == "__main__":
    main()
