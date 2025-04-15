clc;
clear;

%% 参数设置
a = 7.5; b = 7.5; c = 9;      % 平台尺寸
container_size = 60;         % 容器边长

% 定义容器顶点 CV (3×8) —— 假设容器为立方体，顶点坐标如下：
cv1 = [container_size; container_size; container_size];
cv2 = [container_size; 0; container_size];
cv3 = [0; 0; container_size];
cv4 = [0; container_size; container_size];
cv5 = [container_size; container_size; 0];
cv6 = [container_size; 0; 0];
cv7 = [0; 0; 0];
cv8 = [0; container_size; 0];
CV = [cv1, cv2, cv3, cv4, cv5, cv6, cv7, cv8];

% 假设测量得到的绳长 Q（8×1），单位与 CV 一致（例如 cm 或 m）
Q = [40; 42; 41; 43; 39; 40; 41; 42];

% 初始估计平台位姿 [x; y; z; roll; pitch; yaw]（单位：m, rad）
initial_pose = [30; 30; 30; 0; 0; 0];

% 设置 fsolve 选项
options = optimoptions('fsolve','Display','iter','TolFun',1e-6,'TolX',1e-6);

% 使用 fsolve 求解 FK 方程组
solution = fsolve(@(pose) fk_equations(pose, Q, CV, a, b, c), initial_pose, options);

disp('Actual pose');
disp(initial_pose);

disp('Solved platform pose:');
disp(solution);
