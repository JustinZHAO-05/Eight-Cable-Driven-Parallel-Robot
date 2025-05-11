function [trajs_q , v_q, a_q] =config_space_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,...
                                  start_acceleration, target_acceleration, T, t_vals)

    
    [wj,wv,wa] = workspace_trajectory_planning(start_pose, target_pose, start_velocity, target_velocity,...
                                  start_acceleration, target_acceleration, T, t_vals);

    workspace_traj = [wj,wv,wa];

    %disp(workspace_traj);

    J = Jaco(wj);
   % disp (J);

    trajs_q = calculateCableLengths(workspace_traj(1,1),workspace_traj(2,1),workspace_traj(3,1),workspace_traj(4,1),workspace_traj(5,1),workspace_traj(6,1));

    v_q = J * wv;

    a_q = computeCableAcceleration(wj,wv,wa);

    

    disp(trajs_q);
    disp(v_q);
    disp(a_q);

    %输入为     6个列向量（长度单位是cm，角度单位是rad），总共的轨迹时长(T)单位s，，你想要知道的那一个时刻（t_vals)单位s
    %输出为在t_vals时刻的绳长，绳长速度，绳长加速度，单位同上