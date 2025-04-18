function [r, alpha, phi] = cart2sph_custom(vx, vy, vz)
%CART2SPH_CUSTOM  笛卡尔坐标转球坐标（弧度制）
%   [r, alpha, phi] = cart2sph_custom(vx, vy, vz)
%   输入：
%     vx, vy, vz — 三维分量（同维数组或标量）
%   输出：
%     r     — 向量长度
%     alpha — 仰角 elevation (rad)，在 [-pi/2, +pi/2] 范围
%     phi   — 方位角 azimuth (rad)，取 atan2(vy,vx) 结果，范围 (-pi, +pi]
%
%   计算步骤：
%     r     = sqrt(vx.^2 + vy.^2 + vz.^2);
%     alpha = asin(vz ./ r);
%     phi   = atan2(vy, vx);
%   注意：当 r==0 时，alpha 和 phi 可设为 0。
%

    r = sqrt(vx.^2 + vy.^2 + vz.^2);
    % 防止除以零
    zeroMask = (r == 0);
    alpha = asin( vz ./ max(r, eps) );
    phi   = atan2(vy, vx);
    % 对于零长度向量，统一定义角度为 0
    alpha(zeroMask) = 0;
    phi(zeroMask)   = 0;
end
