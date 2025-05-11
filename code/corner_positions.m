function WAV=corner_positions(varargin)

    if nargin < 6
        error('Not enough input arguments. At least x, y, z, alpha, beta, gamma must be provided.');
    elseif nargin < 9
        % Only 6 arguments provided: assume they are [x, y, z, alpha, beta, gamma]
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
        a = varargin{1};
        b = varargin{2};
        c = varargin{3};
        x = varargin{4};
        y = varargin{5};
        z = varargin{6};
        alpha = varargin{7};
        beta = varargin{8};
        gamma = varargin{9};
    end


% Compute half-dimensions of the platform (used to determine mounting points)
hl = a/2;
hw = b/2;
hh = c/2;




%% Define platform mounting points relative to the platform center (MAV)
% The platform is assumed to be a rectangular box.
av1 = [hl;  hw-1.5;  hh];
av2 = [hl; -hw+1.5;  hh];
av3 = [-hl; -hw+1.5;  hh];
av4 = [-hl;  hw-1.5;  hh];
av5 = [hl;  hw; -hh];
av6 = [hl; -hw; -hh];
av7 = [-hl; -hw; -hh];
av8 = [-hl;  hw; -hh];

% MAV is a 3x8 matrix; each column is the mounting point (relative vector)
MAV = [av5, av6, av7, av8, av1, av2, av3, av4];


%% Build the platform pose transformation matrix
% WTM: Translation vector as a column vector
WTM = [x; y; z];

% WRM: Rotation transformation from Euler angles using our custom eul2rotm
WRM = eul2rotm([alpha, beta, gamma], 'ZYX');

% The homogeneous transformation for the platform
pose = trvec2tform(WTM) * WRM;

%% Transform the platform mounting points to world coordinates
% Extend MAV to homogeneous coordinates (add a row of ones)
exMAV = [MAV; ones(1,8)];
% Transform: WAV = pose * exMAV, then take the first 3 rows
exWAV = pose * exMAV;
WAV = exWAV(1:3, :);
