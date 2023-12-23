%% simulation
L=0.26;                                % height of com
g=9.8;                                 % acceleration of gravity
ddxy_s_max = [0.5;1.5];        % maximal up bound of surface moving acceleration (ax,ay)
ddxy_s_min = [-0.5;-1.5];      % minimum low bound of surface moving acceleration (ax,ay)
dT = 0.01;                            % simulation prediction time step
T_s = 2.5;                               % simulation time
from t=0.4s to t=2.5s         % period of surface moving acceleration
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
j_max = [1,2];
j_min = [-1,-2];

%% code
% moving surface
Ax = 0.005;
T_periodx = 0.6;
Ay = 0.02;
T_periody = 0.6;

if i >= 42
        ddxy_s = [-Ax*2*pi/T_periodx*2*pi/T_periodx*sin((current_T-T(41))*2*pi/T_periodx);
                  -Ay*2*pi/T_periody*2*pi/T_periody*sin((current_T-T(41))*2*pi/T_periody)];
    else
        ddxy_s = [0;0];
    end