function R = eul2rotm(eul, order)
%EUL2ROTM Convert Euler angles to a homogeneous rotation matrix.
%
%   R = EUL2ROTM(EUL) converts Euler angles EUL = [yaw, pitch, roll] (in radians)
%   to a 4x4 homogeneous rotation matrix R using the ZYX rotation order:
%
%       R = R_z(yaw) * R_y(pitch) * R_x(roll)
%
%   If ORDER is not provided, the default is 'ZYX'.
%
%   Example:
%       R = eul2rotm([pi/4, pi/6, pi/3])
%
%   Note: Only the 'ZYX' order is implemented.

    if nargin < 2
        order = 'ZYX';
    end

    if numel(eul) ~= 3
        error('Input Euler angles must be a 3-element vector [yaw, pitch, roll].');
    end

    eul = eul(:);  % Ensure column vector

    switch upper(order)
        case 'ZYX'
            psi   = eul(1);  % yaw
            theta = eul(2);  % pitch
            phi   = eul(3);  % roll

            % Rotation about Z-axis (yaw)
            Rz = [ cos(psi), -sin(psi), 0, 0;
                   sin(psi),  cos(psi), 0, 0;
                         0,         0,  1, 0;
                         0,         0,  0, 1];

            % Rotation about Y-axis (pitch)
            Ry = [ cos(theta), 0, sin(theta), 0;
                         0,    1,      0,     0;
                  -sin(theta), 0, cos(theta), 0;
                         0,    0,      0,     1];

            % Rotation about X-axis (roll)
            Rx = [ 1,     0,         0,    0;
                   0, cos(phi), -sin(phi), 0;
                   0, sin(phi),  cos(phi), 0;
                   0,    0,         0,    1];

            R = Rz * Ry * Rx;
        otherwise
            error('Currently, only ''ZYX'' rotation order is implemented.');
    end
end
