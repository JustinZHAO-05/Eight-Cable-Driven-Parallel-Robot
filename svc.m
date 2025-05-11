% 假设平台位姿 x = [x; y; z; roll; pitch; yaw]，示例取某一位姿
x0 = [0.3; 0.4; 0.2; 0.1; 0; 0.2];

% 定义一个小的扰动量
delta = 1e-4;

% 初始化雅可比矩阵 J (8x6)
J = zeros(8, 6);

% 假设你有一个函数 cableLength(x) 返回 8x1 绳长向量
s0 = calculateCableLengths(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));

% 数值求雅可比: 对 x0 中的每个分量加上一个小扰动，计算对应变化
for i = 1:6
    x_temp = x0;
    x_temp(i) = x_temp(i) + delta;
    s_temp = calculateCableLengths(x_temp(1),x_temp(2),x_temp(3),x_temp(4),x_temp(5),x_temp(6));
    J(:, i) = (s_temp - s0) / delta;
end

% 计算雅可比的奇异值
[~, S, ~] = svd(J);
singular_values = diag(S);

disp('雅可比矩阵 J 的奇异值:');
disp(singular_values);

% 你可以绘制最小奇异值在工作空间内的变化曲线（需要遍历不同位姿）
