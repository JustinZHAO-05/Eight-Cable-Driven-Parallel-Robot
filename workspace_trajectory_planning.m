function [trajs , v, a] =workspace_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,...
                                  start_acceleration, target_acceleration, T, t_vals)

                 
    
    trajs = [0;0;0;0;0;0];
    v = [0;0;0;0;0;0];
    a = [0;0;0;0;0;0];

    for i = 1:6

        [trajs(i),v(i),a(i)] = quintic_trajectory(start_pose(i), target_pose(i), start_velocity(i), target_velocity(i), start_acceleration(i), target_acceleration(i), T, t_vals) ;
    
    end

    %输入为     6个列向量（长度单位是cm，角度单位是rad），总共的轨迹时长(T)单位s，，你想要知道的那一个时刻（t_vals)单位s
    %输出为在t_vals时刻的位姿列向量，速度列向量，加速度列向量，单位同上

    