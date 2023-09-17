%% simulation
L=0.26;                                % height of com
g=9.8;                                 % acceleration of gravity
ddxy_s_max = [0.8;0.5];        % maximal up bound of surface moving acceleration (ax,ay)
ddxy_s_min = [-0.8;-0.5];      % minimum low bound of surface moving acceleration (ax,ay)
dT = 0.01;                            % simulation prediction time step
T_s = 2;                               % simulation time
from t=0.28s to t=1.28s         % period of surface moving acceleration with (0.57;0.3) m/(s^2)
time_stop = 0.88s                % MPC horizon beginning moment

%% MPC
horizon = 100
deta = 0.01                          %MPC prediction time step
T_h = 1;                               %MPC prediction time, so that horizon = 100
foot_length = 0.02;              % the length and width of foot
step_width = 0.1;                  % y-direction displacement between two feet during double support
step_size = 0.05;                  % x-direction displacement between two feet during double support
single_support_time = 0.2;    % single support time
double_support_time = 0.1;  % double support time
gait_time = single_support_time + double_support_time ;

%% contingency parameters
j_max = [1,1];
j_min = [-1,-1];

%% code
if i >= 30 && i <= 130
        ddxy_s = [0.57;0.3]; % 0.56  0.58
    else
        ddxy_s = [0;0];
    end

