function [J_inv, sv_inv, condJ_inv] = computeInverseJacobian(x0, delta)
% COMPUTEINVERSEJACOBIAN  从配置空间到工作空间的伪逆雅可比 + 奇异值 + 条件数
%   输入:
%     x0    - 6×1 工作空间位姿 [x; y; z; roll; pitch; yaw]
%     delta - 差分步长 (可选, 默认 1e-6)
%   输出:
%     J_inv      - 6×8 伪逆雅可比
%     sv_inv     - 6×1 奇异值 (从大到小)
%     condJ_inv  - 条件数 = max(sv_inv)/min(sv_inv)

    if nargin < 2
        delta = 1e-6;
    end

    % 1) 先数值计算逆运动学雅可比 J (8x6)
    %    这里假设有函数 computeJacobianNumeric(x0, delta)：
    %      [J, ~, ~] = computeJacobianNumeric(x0, delta);
    [J, ~, ~] = computeJacobianNumeric(x0, delta);

    % 2) 计算伪逆，映射 dq -> dx
    J_inv = pinv(J);  % 等价于 (J' * J) \ J'

    % 3) SVD 分解得到奇异值
    [~, S, ~] = svd(J_inv);
    sv_inv = diag(S);

    % 4) 条件数 = 最大奇异值 / 最小奇异值
    condJ_inv = sv_inv(1) / sv_inv(end);
    % 或者直接： condJ_inv = cond(J_inv);

end
