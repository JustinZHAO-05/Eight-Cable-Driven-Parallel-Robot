function T_cable = cable_tension(x, y, z, roll, pitch, yaw, a_dyn, euler_dot, euler_ddot)
% CABLE_TENSION  Calculates the tensions in eight cables for a 6-DOF,
% 8-cable-driven parallel robot.
%
%   T_cable = cable_tension(x, y, z, roll, pitch, yaw, a_dyn, euler_dot, euler_ddot)
%
%   Inputs:
%       x, y, z         : Coordinates of the platform's center (in meters)
%       roll, pitch, yaw: Euler angles (in radians), where roll = φ, pitch = θ, yaw = ψ.
%       a_dyn           : Platform linear acceleration (m/s^2); e.g., [1; -2; 0.0]
%       euler_dot       : Euler angular velocity vector [dot_phi; dot_theta; dot_psi] (rad/s)
%       euler_ddot      : Euler angular acceleration vector [ddot_phi; ddot_theta; ddot_psi] (rad/s^2)
%
%  输入很抽象！！！！   x (m), y(m), z(m), roll(rad)(绕x轴）, pitch（绕y轴）,yaw（绕z轴），平动加速度列向量[a_x;a_y;a_z](m/s^2),欧拉角速度列向量[v_roll;v_pitch;v_yaw](rad/s),欧拉角加速度列向量[a_roll;a_pitch;a_yaw](rad/s^2)
%
% 输出为列向量，有上到下为八根绳的张力
%
%   The function uses default values for other parameters:
%       Platform mass, m = 1.2 kg
%       Gravitational acceleration, g = [0; 0; -9.81] m/s^2
%       Inertia matrix, I = diag([0.5, 0.5, 1]) kg·m^2
%       Container side length = 0.6 m
%       Platform mounting geometry is defined via r_platform.
%
%   The algorithm consists of three main steps:
%       1. Compute the external force (F_ext) and external moment (M_ext) acting on
%          the platform using the platform's linear and angular dynamics.
%       2. Construct the equilibrium matrix A (6x8) based on the cable unit direction
%          vectors and the moment arms (cross product of mounting point position and u).
%       3. Solve the equilibrium equation A*T = W using regularized least squares
%          to obtain the cable tensions T_cable.
%
%   Output:
%       T_cable: An 8x1 double vector of cable tensions (in Newtons).

    %% ----------------------------
    %% Parameter Setup (Platform Dynamics)
    m = 1.2;                  % Platform mass (kg)
    g = [0; 0; -9.81];        % Gravitational acceleration (m/s^2)
    I = diag([1.2314, 1.219531, 0.740019]);   % Inertia matrix of the platform (kg·m^2)
    
    % a_dyn, euler_dot, euler_ddot are provided as input parameters.
    
    %% ----------------------------
    %% Step 1: Calculate External Force and Moment on the Platform
    % Compute the external translational force:
    % F_ext = m * (platform acceleration + gravitational acceleration)
    F_ext = m * (a_dyn - g);
    
    % Set up Euler angles: roll (phi), pitch (theta), and yaw (psi)
    phi = roll; 
    theta = pitch; 
    psi = yaw;
    
    % Construct the Euler angle conversion matrix (ZYX order)
    % This matrix transforms Euler angle rates into body-fixed angular velocity.
    T_mat = [ 1,        0,            -sin(theta);
              0,  cos(phi),    cos(theta)*sin(phi);
              0, -sin(phi),    cos(theta)*cos(phi)];
    
    % Compute the angular velocity vector (omega)
    omega = T_mat * euler_dot;
    
    % Calculate the partial derivatives of T_mat with respect to phi and theta.
    dT_dphi = [ 0,            0,              0;
                0,      -sin(phi),    cos(theta)*cos(phi);
                0,      -cos(phi),   -cos(theta)*sin(phi)];
    
    dT_dtheta = [ 0,            0,             -cos(theta);
                  0,            0,             -sin(theta)*sin(phi);
                  0,            0,             -sin(theta)*cos(phi)];
    
    % Compute the time derivative of T_mat (T_dot)
    % Here, only the contributions from phi and theta are considered.
    T_dot = dT_dphi * euler_dot(1) + dT_dtheta * euler_dot(2);
    
    % Compute the full angular acceleration (alpha_dyn):
    % alpha_dyn = T_dot * euler_dot + T_mat * euler_ddot
    alpha_dyn = T_dot * euler_dot + T_mat * euler_ddot;
    
    % Compute the external moment using Euler's equation:
    % M_ext = I * alpha_dyn + omega x (I * omega)
    M_ext = I * alpha_dyn + cross(omega, I * omega);
    
    % Combine the translational force and moment into a 6x1 external wrench vector W.
    W = [F_ext; M_ext];
    
    %%----------------以下部分请手动注释掉---------------
    disp('External Force F_ext (N):'); disp(F_ext);
    disp('External Moment M_ext (N·m):'); disp(M_ext);
%%---------------------------------------------------------



    
    %% ----------------------------
    %% Step 2: Construct Equilibrium Matrix A (6x8)
    % Container parameters: set container side length (m)
    container_size_x = 0.68;
    container_size_y = 0.56;
    container_size_z = 0.66;
    
    % Define the container's anchor points (CV) as the eight corners of a cube.
    cv1 = [container_size_x; container_size_y; container_size_z];
    cv2 = [container_size_x; 0; container_size_z];
    cv3 = [0; 0; container_size_z];
    cv4 = [0; container_size_y; container_size_z];
    cv5 = [container_size_x; container_size_y; 0];
    cv6 = [container_size_x; 0; 0];
    cv7 = [0; 0; 0];
    cv8 = [0; container_size_y; 0];
    CV = [cv1, cv2, cv3, cv4, cv5, cv6, cv7, cv8];
    
    % Calculate platform mounting point positions using an auxiliary function.
    % Function "corner_positions" is assumed to compute the eight corner positions
    % of the platform given its center and orientation.
    % The input scaling (multiplying by 100 and then dividing by 100) ensures unit consistency.
    p = corner_positions(x*100, y*100, z*100, yaw, pitch, roll) ./ 100;
    p_mount = [p(:,5),p(:,6),p(:,7),p(:,8),p(:,1),p(:,2),p(:,3),p(:,4)];
    
    % For this analysis, container anchor points B are set equal to CV.
    B = CV;

    r_wheel = 3.6/100 ; 
    
    % Compute the cable direction unit vectors (u) for each cable:
    % u_i = (B_i - p_i) / norm(B_i - p_i)
    u = zeros(3,8);
    for i = 1:8
        former_vec = B(:,i) - p(:,i);
        formerlength = norm(former_vec);
        delta = asin(r_wheel/formerlength);
        [ss,pp,hh] = cart2sph_custom(former_vec(1),former_vec(2),former_vec(3));
        if i <= 4
            newsph = [ss,pp+delta,hh];
        else
            newsph = [ss,pp-delta,hh];
        end
        [xx,yy,zz] = sph2cart_custom(newsph(1),newsph(2),newsph(3));
        diff_vec = [xx,yy,zz];
        u(:,i) = diff_vec / norm(diff_vec);
    end

  %  disp(u);
    
    % Define the platform center (translation vector)
    center = [x; y; z];
    
    % Compute the relative position vector (r) of each mounting point with respect to the platform center:
    r = zeros(size(p));
    for i = 1:8
        r(:,i) = p(:,i) - center;
    end

   % disp(r);
    
    % Construct the equilibrium matrix A (6x8):
    % The top 3 rows consist of the cable direction unit vectors u_i.
    % The bottom 3 rows consist of the cross products r_i x u_i.
    A = zeros(6, 8);
    for i = 1:8
        A(1:3, i) = u(:, i);
        A(4:6, i) = cross(r(:, i), u(:, i));
    end
    
    %% ----------------------------
    %% Step 3: Solve for Cable Tensions using Regularized Least Squares
    % Solve A*T = W for the cable tensions T (an 8x1 vector) using Tikhonov regularization.
    lambda = 1e-3;  % Regularization parameter (adjust as needed)
    %T_cable = (A' * A + lambda * eye(size(A,2))) \ (A' * W);

    % A*T = W, 强制 T >= 0
    %T_cable = lsqnonneg(A, W);

    H = A'*A + lambda*eye(8);
    f = -A'*W;
    % 下界 T >= 0
    lb = ones(8,1);
    %设置lb，可以改变每一根绳上要求的最小的张力

    T_cable = quadprog(H, f, [], [], [], [], lb, []);



    

    
    %disp('Calculated cable tensions (N):');
    %disp(T_cable);
    
    %% ----------------------------以下部分请手动注释掉
    %% (Optional) Plot comparison between actual external wrench and estimated wrench
    W_est = A * T_cable;
    figure;
    subplot(2,1,1);
    stem(W(1:3), 'filled'); title('External Force F_{ext}');
    subplot(2,1,2);
    stem(W(4:6), 'filled'); title('External Moment M_{ext}');
    
    figure;
    subplot(2,1,1);
    stem(W_est(1:3), 'filled'); title('Estimated Force (A*T)');
    subplot(2,1,2);
    stem(W_est(4:6), 'filled'); title('Estimated Moment (A*T)');
    %% ---------------------------------------------------------



    % Ensure output is a double precision 8x1 vector.
    T_cable = double(T_cable);
    disp('Calculated cable tensions (N):');
    disp(T_cable);
end


%  输入很抽象！！！！   x (m), y(m), z(m), roll(rad)(绕x轴）, pitch（绕y轴）,yaw（绕z轴），平动加速度列向量[a_x;a_y;a_z](m/s^2),欧拉角速度列向量[v_roll;v_pitch;v_yaw](rad/s),欧拉角加速度列向量[a_roll;a_pitch;a_yaw](rad/s^2)
%
% 输出为列向量，有上到下为八根绳的张力