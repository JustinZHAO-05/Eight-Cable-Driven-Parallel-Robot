function Q = calculateCableLengths_new(varargin)
% CALCULATECABLELENGTHS Computes the eight cable lengths for a 6-DOF, 8-cable
% driven parallel robot.
%
%   Q = CALCULATECABLELENGTHS(container_size, a, b, c, x, y, z, alpha, beta, gamma)
%   computes the cable lengths based on the provided parameters.
%
%   If only 6 arguments are provided, the first four (container_size, a, b, c)
%   default to 60, 7.5, 7.5, and 9 respectively.
%
%   Inputs:
%       container_size : Side length of the container (default: 60)
%       a              : Platform length (default: 7.5)
%       b              : Platform width (default: 7.5)
%       c              : Platform height (default: 9)
%       x, y, z        : Platform center coordinates
%       alpha          : Roll angle (rad)
%       beta           : Pitch angle (rad)
%       gamma          : Yaw angle (rad)
%
%   Output:
%       Q              : An 8x1 vector of cable lengths (each element corresponds
%                        to one cable's length), computed using the derived formulas.
%


    % Determine the number of input arguments and assign default values if needed.
    if nargin < 6
        error('Not enough input arguments. At least x, y, z, alpha, beta, gamma must be provided.');
    elseif nargin < 12
        % Only 6 arguments provided: assume they are [x, y, z, alpha, beta, gamma]
        container_size_x = 68;
        container_size_y = 56;
        container_size_z = 66;
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
        container_size_x = varargin{1};
        container_size_y = varargin{2};
        container_size_z = varargin{3};
        a = varargin{4};
        b = varargin{5};
        c = varargin{6};
        x = varargin{7};
        y = varargin{8};
        z = varargin{9};
        alpha = varargin{10};
        beta = varargin{11};
        gamma = varargin{12};
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute Cable Lengths for Each Cable Using the Derived Formulas
    % The formulas below compute the Euclidean distance between the container's
    % anchor point and the platform's mounting point for each cable.
    
    % Cable 1:
    Term1_x = container_size_x - x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 ...
              + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
              - (a*cos(alpha)*cos(beta))/2;
    Term1_y = y - container_size_y + (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 ...
              + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
              + (a*cos(beta)*sin(alpha))/2;
    Term1_z = container_size_z - z + (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 ...
              - (b*cos(beta)*sin(gamma))/2;
    Q1 = sqrt(Term1_x^2 + Term1_y^2 + Term1_z^2-3.6^2);

    % Cable 2:
    Term2_x = x - container_size_x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 ...
              - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
              + (a*cos(alpha)*cos(beta))/2;
    Term2_y = container_size_z - z + (a*sin(beta))/2 ...
              + (c*cos(beta)*cos(gamma))/2 ...
              + (b*cos(beta)*sin(gamma))/2;
    Term2_z = y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 ...
              + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
              + (a*cos(beta)*sin(alpha))/2;
    Q2 = sqrt(Term2_x^2 + Term2_y^2 + Term2_z^2-3.6^2);

    % Cable 3:
    Term3_x = container_size_z - z - (a*sin(beta))/2 + (c*cos(beta)*cos(gamma))/2 + (b*cos(beta)*sin(gamma))/2;
    Term3_y = x + (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 ...
        - (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        - (a*cos(alpha)*cos(beta))/2;
    Term3_z = y - (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 ...
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        - (a*cos(beta)*sin(alpha))/2;
    Q3 = sqrt(Term3_x^2 + Term3_y^2 + Term3_z^2-3.6^2);

    % Cable 4:
    Term4_x = y - container_size_y + (b*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)))/2 ...
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        - (a*cos(beta)*sin(alpha))/2;
    Term4_y = z - container_size_z + (a*sin(beta))/2 ...
        - (c*cos(beta)*cos(gamma))/2 ...
        + (b*cos(beta)*sin(gamma))/2;
    Term4_z = (b*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)))/2 ...
        - x + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        + (a*cos(alpha)*cos(beta))/2;
    Q4 = sqrt(Term4_x^2 + Term4_y^2 + Term4_z^2-3.6^2);

    % Cable 5:
    Term5_x = z - (a*sin(beta))/2 ...
        + cos(beta)*sin(gamma)*(b/2 - 3/2) ...
        + (c*cos(beta)*cos(gamma))/2;
    Term5_y = x - container_size_x - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) ...
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        + (a*cos(alpha)*cos(beta))/2;
    Term5_z = y - container_size_y + (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) ...
        - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        + (a*cos(beta)*sin(alpha))/2;
    Q5 = sqrt(Term5_x^2 + Term5_y^2 + Term5_z^2-3.6^2);

    % Cable 6:
    Term6_x = y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) ...
        - (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        + (a*cos(beta)*sin(alpha))/2;
    Term6_y = z - (a*sin(beta))/2 ...
        - cos(beta)*sin(gamma)*(b/2 - 3/2) ...
        + (c*cos(beta)*cos(gamma))/2;
    Term6_z = x - container_size_x + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) ...
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        + (a*cos(alpha)*cos(beta))/2;
    Q6 = sqrt(Term6_x^2 + Term6_y^2 + Term6_z^2-3.6^2);

    % Cable 7:
    Term7_x = x + (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) ...
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        - (a*cos(alpha)*cos(beta))/2;
    Term7_y = z + (a*sin(beta))/2 ...
        - cos(beta)*sin(gamma)*(b/2 - 3/2) ...
        + (c*cos(beta)*cos(gamma))/2;
    Term7_z = (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) ...
        - y + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        + (a*cos(beta)*sin(alpha))/2;
    Q7 = sqrt(Term7_x^2 + Term7_y^2 + Term7_z^2-3.6^2);

    % Cable 8:
    Term8_x = x - (b/2 - 3/2)*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) ...
        + (c*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)))/2 ...
        - (a*cos(alpha)*cos(beta))/2;
    Term8_y = z + (a*sin(beta))/2 + cos(beta)*sin(gamma)*(b/2 - 3/2) + (c*cos(beta)*cos(gamma))/2;
    Term8_z = container_size_y - y - (b/2 - 3/2)*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) ...
        + (c*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)))/2 ...
        + (a*cos(beta)*sin(alpha))/2;
    Q8 = sqrt(Term8_x^2 + Term8_y^2 + Term8_z^2-3.6^2);

    % Combine the eight cable lengths into an 8x1 column vector
    Q = [Q1; Q2; Q3; Q4; Q5; Q6; Q7; Q8];
end