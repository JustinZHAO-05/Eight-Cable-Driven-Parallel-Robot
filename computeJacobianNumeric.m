function [J, singular_values, condJ] = computeJacobianNumeric(x0, delta)
% COMPUTEJACOBIANNUMERIC  数值求雅可比并计算奇异值与条件数
%   [J, SV, condJ] = computeJacobianNumeric(x0, delta)
%   Inputs:
%     x0    - 6×1 向量 [x; y; z; roll; pitch; yaw]
%     delta - 差分步长 (可选, 默认 1e-6)
%   Outputs:
%     J                - 8×6 雅可比矩阵 ∂s/∂x
%     singular_values  - 6×1 奇异值，从大到小排序
%     condJ            - 条件数 = σ_max / σ_min

    if nargin < 2
        delta = 1e-8;
    end

    % 原始绳长
    s0 = calculateCableLengths(...
        x0(1), x0(2), x0(3), x0(4), x0(5), x0(6) );

    n = numel(x0);       % =6
    J = zeros(8, n);

    % 中心差分
    for i = 1:n
        dx = zeros(n,1);
        dx(i) = delta;
        % f(x0 + dx)
        sp = calculateCableLengths(...
            x0(1)+dx(1), x0(2)+dx(2), x0(3)+dx(3), ...
            x0(4)+dx(4), x0(5)+dx(5), x0(6)+dx(6));
        % f(x0 - dx)
        sm = calculateCableLengths(...
            x0(1)-dx(1), x0(2)-dx(2), x0(3)-dx(3), ...
            x0(4)-dx(4), x0(5)-dx(5), x0(6)-dx(6));
        % 差分商
        J(:, i) = (sp - sm) / (2*delta);
    end

    % SVD 分解
    [~, S, ~] = svd(J);
    singular_values = diag(S);
    condJ = singular_values(1) / singular_values(end);
end
