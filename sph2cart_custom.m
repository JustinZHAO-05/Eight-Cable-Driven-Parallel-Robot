function [vx, vy, vz] = sph2cart_custom(r, alpha, phi)
%SPH2CART_CUSTOM  球坐标转笛卡尔坐标（弧度制）
%   [vx, vy, vz] = sph2cart_custom(r, alpha, phi)
%   输入：
%     r     — 向量长度（标量或同维数组）
%     alpha — 仰角 elevation (rad)，定义为向量与 xy 平面的夹角，正值表示向量向上
%     phi   — 方位角 azimuth (rad)，定义为在 xy 平面上投影相对于 x 轴的角度
%   输出：
%     vx, vy, vz — 对应的三维分量
%
%   公式：
%     vx = r * cos(alpha) * cos(phi);
%     vy = r * cos(alpha) * sin(phi);
%     vz = r * sin(alpha);
%

    vx = r .* cos(alpha) .* cos(phi);
    vy = r .* cos(alpha) .* sin(phi);
    vz = r .* sin(alpha);
end
