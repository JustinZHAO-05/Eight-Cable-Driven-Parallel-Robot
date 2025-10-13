            start_pose = [34, 28, 33, 0, 0, 0]
            target_pose = [38, 38, 38, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity = [0]*6
            target_velocity = [0]*6
            start_acceleration = [0]*6
            target_acceleration = [0]*6
            # T is the total motion time (seconds), num_samples is the number of trajectory points

            start_pose_2 = target_pose
            target_pose_2 = [25, 25, 25, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_2 = target_velocity
            target_velocity_2 = [0]*6
            start_acceleration_2 = target_acceleration
            target_acceleration_2 = [0]*6

            pose2,traj2,sample_time2,num_sample2=trajectory_workspace_serial(start_pose_2, target_pose_2, start_velocity_2, target_velocity_2,
                                         start_acceleration_2, target_acceleration_2, 10, 100)
            
            start_pose_3 = target_pose_2
            target_pose_3 = [38, 38, 15, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_3 = target_velocity_2
            target_velocity_3 = [0]*6
            start_acceleration_3 = target_acceleration_2
            target_acceleration_3 = [0]*6

            pose3,traj3,sample_time3,num_sample3=trajectory_workspace_serial(start_pose_3, target_pose_3, start_velocity_3, target_velocity_3,
                                         start_acceleration_3, target_acceleration_3, 10, 100)
            
            start_pose_4 = target_pose_3
            target_pose_4 = [38, 38, 38, 0, 0,0]
            # Set initial and target velocities & accelerations (here all zeros)
            start_velocity_4 = target_velocity_3
            target_velocity_4 = [0]*6
            start_acceleration_4 = target_acceleration_3
            target_acceleration_4 = [0]*6

            pose4,traj4,sample_time4,num_sample4=trajectory_workspace_serial(start_pose_4, target_pose_4, start_velocity_4, target_velocity_4,
                                         start_acceleration_4, target_acceleration_4, 10, 100)
            
            
            
            poses=[pose2,pose3,pose4]
            
            trajs =[traj2,traj3,traj4]
            sample_times = [sample_time2,sample_time3,sample_time4]
            num_samples = [num_sample2,num_sample3,num_sample4]

            execute_trajectory_workspace(start_pose, target_pose, start_velocity, target_velocity,
                                         start_acceleration, target_acceleration, 10, 100)
            
            execute_traj_serial(poses,trajs,sample_times,num_samples)