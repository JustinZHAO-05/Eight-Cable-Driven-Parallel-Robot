function T_cable=cable_tension(x,y,z,roll,pitch,yaw,a,euler_dot,euler_ddot)



%% ----------------------------
%% 参数设置（平台与运动学参数）
m = 1.2;                  % 平台质量 (kg)
g = [0; 0; -9.81];      % 重力 (m/s^2)
I = diag([0.5, 0.5, 1]);  % 平台惯性矩阵 (kg·m^2)

% 平台位姿（单位：m, rad）
%x = 3; y = -5; z = 6;
%roll = 0; pitch = 0; yaw = 0;

% 平台线性加速度（m/s^2）
%a = [1; -2; 0.0];

% 欧拉角速率和加速度（rad/s, rad/s^2）
%euler_dot = [0.5; 0.3; 0.4];
%euler_ddot = [0.5; 0.1; -0.2];

%% ----------------------------
%% Step 1: 计算平台外力 F_ext 和外力矩 M_ext（完整考虑 T_dot）
% 平移外力
F_ext = m * a + m * g;

% 欧拉角与转换矩阵（ZYX顺序）
phi = roll; theta = pitch; psi = yaw;
T_mat = [ 1,        0,            -sin(theta);
          0,  cos(phi),    cos(theta)*sin(phi);
          0, -sin(phi),    cos(theta)*cos(phi)];

% 角速度
omega = T_mat * euler_dot;

% 计算 T_mat 关于 phi 和 theta 的偏导数
dT_dphi = [ 0,            0,              0;
            0,      -sin(phi),    cos(theta)*cos(phi);
            0,      -cos(phi),   -cos(theta)*sin(phi) ];

dT_dtheta = [ 0,            0,             -cos(theta);
              0,            0,             -sin(theta)*sin(phi);
              0,            0,             -sin(theta)*cos(phi) ];

% 计算 T_dot：注意只考虑 phi 和 theta 的贡献
T_dot = dT_dphi * euler_dot(1) + dT_dtheta * euler_dot(2);

% 完整计算角加速度：alpha = T_dot*euler_dot + T_mat*euler_ddot
alpha = T_dot * euler_dot + T_mat * euler_ddot;

% 外力矩
M_ext = I * alpha + cross(omega, I * omega);

% 合成外部作用向量
W = [F_ext; M_ext];

disp('外力 F_ext (N):'); disp(F_ext);
disp('外力矩 M_ext (N·m):'); disp(M_ext);

%% ----------------------------
%% Step 2: 构造矩阵 A（6x8），用于平衡方程 A*T = W
% 平台与容器参数
container_size = 0.6;
cv1 = [container_size; container_size; container_size];
cv2 = [container_size; 0; container_size];
cv3 = [0; 0; container_size];
cv4 = [0; container_size; container_size];
cv5 = [container_size; container_size; 0];
cv6 = [container_size; 0; 0];
cv7 = [0; 0; 0];
cv8 = [0; container_size; 0];

% CV is a 3x8 matrix; each column is one corner.
CV = [cv1, cv2, cv3, cv4, cv5, cv6, cv7, cv8];




% 平台挂载点位置
p = corner_positions(x*100,y*100,z*100,yaw,pitch,roll)./100;

% 定义容器锚点（正方体容器的八个角）
B = CV;

% 计算每根绳子的牵引方向单位向量 u
u = zeros(3,8);
for i = 1:8
    diff_vec = B(:,i) - p(:,i);
    u(:,i) = diff_vec / norm(diff_vec);
end

center = [x;y;z];
% r 向量（平台挂载点相对于平台质心）
for i = 1:8
    r(:,i) = p(:,i)-center;
end

% 构造 A 矩阵：上半部分为 u_i， 下半部分为 r_i x u_i
A = zeros(6, 8);
for i = 1:8
    A(1:3, i) = u(:, i);
    A(4:6, i) = cross(r(:, i), u(:, i));
end

%% ----------------------------
%% Step 3: 利用最小二乘法求解张力 T
%%T_cable = pinv(A) * W;  % 采用伪逆求最小二乘解

lambda = 1e-3;  % 正则化参数，根据具体情况调整
T_cable = (A'*A + lambda*eye(size(A,2))) \ (A'*W);


disp('求解得到的八根绳子的张力 (N):');
disp(T_cable);

% 补充：绘图显示 W 与 A*T 的比较
W_est = A * T_cable;
figure;
subplot(2,1,1);
stem(W(1:3), 'filled'); title('外力 F_{ext}');
subplot(2,1,2);
stem(W(4:6), 'filled'); title('外力矩 M_{ext}');

figure;
subplot(2,1,1);
stem(W_est(1:3), 'filled'); title('A*T: 估计外力');
subplot(2,1,2);
stem(W_est(4:6), 'filled'); title('A*T: 估计外力矩');

