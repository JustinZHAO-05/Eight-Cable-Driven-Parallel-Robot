function [J_inv, singular_vals, cond_number] = analyzeTenseJacobianNonDim(x0,  delta)
% ANALYZEJACOBIANNONDIM  归一化+伪逆雅可比 & 奇异值 & 条件数

%张力的雅可比，奇异值与条件数
%
%   [J_inv, singular_vals, cond_number] = ...
%       analyzeJacobianNonDim(x0, delta)
%
%   输入：
%     x0        - 6×1 向量 [x; y; z; roll; pitch; yaw]（工作空间位姿）(m,m,m,rad,rad,rad)

%     delta     - （可选）数值差分步长，默认 1e-6
%
%   输出：
%     J_inv        - 6×8 伪逆雅可比（非量纲化后的）
%     singular_vals - 6×1 伪逆雅可比的奇异值（降序）
%     cond_number  - 条件数 = max(singular_vals)/min(singular_vals)
%
%   备注：
%     1) 本函数依赖于 calculateCableLengths(x,y,z,roll,pitch,yaw)，
%        它返回 8×1 的绳长向量 q。
%     2) 非量纲化采用 x_nd = [x/L; angles/L_theta]。

    if nargin<4 || isempty(delta)
        delta = 1e-6;
    end

    % 1) 数值差分计算 J = ∂q/∂x  (8×6)
    % s0 = calculateCableLengths( x0(1), x0(2), x0(3), x0(4), x0(5), x0(6) );
    J  = zeros(8,6);

    
    for i = 1:6
        x1_temp    = x0;
        
        x1_temp(i) = x1_temp(i) + delta;
        
        s1_temp    = cable_tension( ...
                      x1_temp(1), x1_temp(2), x1_temp(3), ...
                      x1_temp(4), x1_temp(5), x1_temp(6),[0;0;0],[0;0;0],[0;0;0] )  ;

        x2_temp    = x0;
         
        x2_temp(i) = x2_temp(i) - delta;
        
        s2_temp    = cable_tension( ...
                      x2_temp(1), x2_temp(2), x2_temp(3), ...
                      x2_temp(4), x2_temp(5), x2_temp(6),[0;0;0],[0;0;0],[0;0;0] )  ;

        J(:,i)    = (s1_temp - s2_temp) / (2*delta);
    end
    
    disp(J);
    
    L1 = 0.075;
    L2 = 0.09;
    LC = 0.6;
    L_theta = sqrt(2*(L1^2)+L2^2)/2;
    % 2) 构造非量纲化缩放矩阵 S_inv = diag(L, L, L, L_theta, L_theta, L_theta)
    S_inv = diag([L1/LC, L1/LC, L2/LC, L_theta, L_theta, L_theta]);

    % 3) 非量纲化雅可比：J_nd = J * S_inv
    J_nd = J * S_inv;

    disp(J_nd);

    % 4) 计算伪逆：J_inv (6×8)
    J_inv = pinv(J_nd);

    disp(J_inv);

    % 5) 奇异值分解
    [~, S, ~]       = svd(J_inv);
    singular_vals  = diag(S);        % 从大到小排列

    % 6) 条件数
    cond_number = singular_vals(1) / singular_vals(end);
end


%对张力求雅可比与奇异值，仅供参考，