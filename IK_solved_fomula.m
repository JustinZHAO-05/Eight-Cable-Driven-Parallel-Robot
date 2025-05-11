function Q=IK_solved_fomula(varargin)


% CALCULATECABLELENGTHS Calculates the cable lengths for an 8-cable driven 
% 6-DOF parallel robot.
%
%   Q = CALCULATECABLELENGTHS(container_size, a, b, c, x, y, z, alpha, beta, gamma)
%
%   Inputs:
%       container_size: Side length of the cubic container (default: 60)
%       a: Platform length (default: 7.5)
%       b: Platform width (default: 7.5)
%       c: Platform height (default: 9)
%       x, y, z: Coordinates of the platform center in the workspace
%       alpha: Roll angle (radians)
%       beta:  Pitch angle (radians)
%       gamma: Yaw angle (radians)
%
%   Output:
%       Q: An 8x1 vector of cable lengths (each element corresponds to one cable)
%
%   The function computes the cable lengths as the Euclidean distances 
%   between the container's anchor points (the 8 corners of a cube) and 
%   the transformed platform mounting points (assumed to be at the 8 corners 
%   of a rectangular box with dimensions a, b, c), where the platform pose 
%   is given by [x, y, z, alpha, beta, gamma].
%
%   Note: All angles are in radians.

    % Set default values for the first four parameters if not provided.
    if nargin < 6
        error('Not enough input arguments. At least x, y, z, alpha, beta, gamma must be provided.');
    elseif nargin < 10
        % Only 6 arguments provided: assume they are [x, y, z, alpha, beta, gamma]
        container_size = 60;
        a = 7.5;
        b = 7.5;
        c = 9;
        x = varargin{1};
        y = varargin{2};
        z = varargin{3};
        alpha = varargin{4};
        beta = varargin{5};
        gamma = varargin{6};
    else
        % All 10 arguments provided
        container_size = varargin{1};
        a = varargin{2};
        b = varargin{3};
        c = varargin{4};
        x = varargin{5};
        y = varargin{6};
        z = varargin{7};
        alpha = varargin{8};
        beta = varargin{9};
        gamma = varargin{10};
    end

    Q = [
        ((container_size - x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 - (a*cos(alpha)*cos(beta))/2)^2 + (y - container_size + (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2 + (container_size - z + (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 - (b*cos(beta)*sin(gamma))/2)^2)^(1/2);
                  ((x - container_size + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 + (a*cos(alpha)*cos(beta))/2)^2 + (container_size - z + (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 + (b*cos(beta)*sin(gamma))/2)^2 + (y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2)^(1/2);
                                   ((container_size - z - (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 + (b*cos(beta)*sin(gamma))/2)^2 + (x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 - (a*cos(alpha)*cos(beta))/2)^2 + (y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 - (a*cos(beta)*sin(alpha))/2)^2)^(1/2);
                  ((y - container_size + (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 - (a*cos(beta)*sin(alpha))/2)^2 + (z - container_size + (a*sin(beta))/2 - (c*cos(beta)*cos(gamma))/2 + (b*cos(beta)*sin(gamma))/2)^2 + ((b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 - x + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 + (a*cos(alpha)*cos(beta))/2)^2)^(1/2);
((z - (a*sin(beta))/2 + cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2)^2 + (x - container_size - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 + (a*cos(alpha)*cos(beta))/2)^2 + (y - container_size + (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2)^(1/2);
                 ((y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2 + (z - (a*sin(beta))/2 - cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2)^2 + (x - container_size + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 + (a*cos(alpha)*cos(beta))/2)^2)^(1/2);
                                  ((x + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 - (a*cos(alpha)*cos(beta))/2)^2 + (z + (a*sin(beta))/2 - cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2)^2 + ((b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - y + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2)^(1/2);
                 ((x - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 - (a*cos(alpha)*cos(beta))/2)^2 + (z + (a*sin(beta))/2 + cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2)^2 + (container_size - y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 + (a*cos(beta)*sin(alpha))/2)^2)^(1/2);]