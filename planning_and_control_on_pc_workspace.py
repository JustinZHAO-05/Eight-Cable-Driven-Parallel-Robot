#!/usr/bin/env python3
import serial
import numpy as np
import time
from IK_calculation import calculate_cable_lengths 
# -------------------------------
# Global Variables: Robot State
# -------------------------------
# current_pose: [x, y, z, roll, pitch, yaw]
current_pose = [0, 0, 0, 0, 0, 0] #cm
# current_lengths: cable lengths for 8 cables
current_lengths = [0] * 8  

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

def compute_motor_steps(current_lengths, target_lengths, step_angle=1.8, reduction_ratio=1, drum_radius=3.5): #cm
    """
    Compute the number of steps required for each motor based on the change in cable length.
    Returns a list of 8 integer step values.
    """
    steps = []
    for i in range(8):
        delta_s = target_lengths[i] - current_lengths[i]  # Change in cable length
        steps_per_revolution = 360 / step_angle
        length_per_rev = 2 * np.pi * drum_radius
        delta_steps = (delta_s / length_per_rev) * steps_per_revolution * reduction_ratio
        steps.append(int(delta_steps))
    return steps

# Initialize serial port; adjust COM port and baud rate as needed
ser = serial.Serial('COM7', 115200)

def send_motor_steps(steps, sample_time):
    """
    Convert the step command to a string and send it over serial to Arduino.
    The command string now includes the sample_time information.
    Example format: "M 500 -300 200 0 100 0 -150 250;T:80\n"
    where 80 (ms) is the time interval between samples.
    """
    command = "M " + " ".join(map(str, steps)) + ";T:" + str(sample_time) + "\n"
    ser.write(command.encode())

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
    Move the robot to the target cable lengths by computing the required steps,
    sending the command (including sample_time information) to Arduino,
    and updating the global current_lengths.
    """
    global current_lengths
    steps = compute_motor_steps(current_lengths, target_lengths)
    send_motor_steps(steps, sample_time)
    time.sleep(sample_time / 1000.0)
    current_lengths = target_lengths

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

    # Move to the starting pose
    s_start = inverse_kinematics(start_pose)
    # Compute sample time (in ms) between trajectory points
    sample_time = int((T * 1000) / num_samples)
    move_to_target(s_start, T0)
    current_pose = start_pose.copy()
    print("\nCurrent pose:", current_pose)
    print("Current cable lengths:", current_lengths)
    
    # Generate workspace trajectory (each point is a 6D pose)
    pose_traj = workspace_trajectory_planning(start_pose, target_pose, start_velocity,
                                              target_velocity, start_acceleration, target_acceleration,
                                              T, num_samples)
    # For each pose point, compute IK and move
    con_traj = []

    for pose in pose_traj:
        con_traj.append(inverse_kinematics(pose))

    for i in range(num_samples):
        move_to_target(con_traj[i], sample_time)
        current_pose = pose_traj[i]  # Update current pose
        print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
        print("Current cable lengths:", current_lengths)

def manual_control(key_input):
    """
    Manual control mode: Adjust the current pose based on keyboard input,
    then compute the new target cable lengths and move to that pose.
    """
    global current_pose
    delta = 1  # Pose increment
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
    # For manual control, use a default sample time (e.g., 50 ms)
    move_to_target(s_target, 50)
    print("\nCurrent pose:", convert_angles_rad_to_deg(current_pose))
    print("Current cable lengths:", current_lengths)

def initialize_robot():
    """
    Initialize the robot by requiring manual placement to the origin,
    then send a "TIGHTEN" command to pre-tension the cables,
    and wait for the Arduino to respond with confirmation.
    """
    input("Please manually move the robot to the origin and press Enter to continue...")
    ser.write(b"TIGHTEN\n")
    print("send")
    response = ser.readline().decode().strip()
    if response == "TIGHTEN_OK":
        print("Cables tightened, initialization complete!")
    else:
        print("Tightening failed, please check the connection!")



import math

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
            start_pose = [30, 20, 10, 75, 0, 0]
            target_pose = [5, 50, 30, 0.05, 60,-60]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity = [0]*6
            target_velocity = [0]*6
            start_acceleration = [0]*6
            target_acceleration = [0]*6
            # T is the total motion time (seconds), num_samples is the number of trajectory points
            execute_trajectory_workspace(start_pose, target_pose, start_velocity, target_velocity,
                                         start_acceleration, target_acceleration, 3, 80)
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
