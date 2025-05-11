function [J_scaled, singular_values, condJ] = computeJacobianScaled(x0, delta)
% COMPUTEJACOBIANSCALED  非量纲化雅可比 + 奇异值 + 条件数
%   [J_scaled, SV, condJ] = computeJacobianScaled(x0, delta, L)
%   Inputs:
%     x0    - 6×1 向量 [x; y; z; roll; pitch; yaw]
%     delta - 差分步长 (可选, 默认 1e-6)
%     L     - 特征长度，用于将「rad→m」
%   Outputs:
%     J_scaled         - 8×6 非量纲化后的雅可比
%     singular_values  - 6×1 奇异值（从大到小）
%     condJ            - 条件数 = max(sv)/min(sv)

    if nargin < 2, delta = 1e-6; end

    L= 0.06955;
    % 1) 数值雅可比（中心差分）
    [J, ~, ~] = computeJacobianNumeric(x0, delta);

    % 2) 非量纲化：把第4–6列乘以特征长度 L
    J_scaled = J;
    J_scaled(:,4:6) = J(:,4:6) * L;

    % 3) SVD 分解
    [~, S, ~] = svd(J_scaled);
    singular_values = diag(S);

    % 4) 条件数，显式取最大/最小
    condJ = max(singular_values) / min(singular_values);
    % 或者直接：
    % condJ = cond(J_scaled);
end
