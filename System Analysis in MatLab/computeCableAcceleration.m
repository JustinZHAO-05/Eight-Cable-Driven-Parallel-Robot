function s_ddot = computeCableAcceleration(x0, x_dot, x_ddot, delta)
% computeCableAcceleration 由工作空间速度/加速度计算配置空间加速度
%   s_ddot = computeCableAcceleration(x0, x_dot, x_ddot, delta)
%   x0    : 6×1 位姿 [x;y;z;roll;pitch;yaw]
%   x_dot : 6×1 位姿速度
%   x_ddot: 6×1 位姿加速度
%   delta : 数值差分步长（缺省 1e-6）
%
% 原理:
%   ṡ     = J(x)  * ẋ
%   s̈     = J(x)  * ẍ  +  J̇(x)*ẋ
%   J̇(x)  = ∑ₖ ∂J/∂xₖ · ẋₖ

    if nargin<4 || isempty(delta)
        delta = 1e-6;
    end

    % 1) 当前位姿下的雅可比 J0 (8×6)
    J0 = Jaco(x0, delta);

    % 2) 计算 J̇(x) = ∑ₖ ∂J/∂xₖ · ẋₖ      (结果仍是 8×6)
    Jdot = zeros(8,6);
    for k = 1:6
        % 在第 k 个分量做正负扰动
        x_p = x0;  x_p(k) = x_p(k) + delta;
        x_m = x0;  x_m(k) = x_m(k) - delta;
        Jp  = Jaco(x_p, delta);    % J(x+Δx_k)
        Jm  = Jaco(x_m, delta);    % J(x−Δx_k)
        dJdxk = (Jp - Jm) / (2*delta);
        % 加权累加
        Jdot = Jdot + dJdxk * x_dot(k);
    end

    % 3) 最终的配置空间加速度 s̈ = J0*ẍ + Jdot*ẋ
    s_ddot = J0 * x_ddot + Jdot * x_dot;
end

