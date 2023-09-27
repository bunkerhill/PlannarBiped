%% simulation
L=0.26;                                % height of com
g=9.8;                                 % acceleration of gravity
ddxy_s_max = [1.15;1.15];        % maximal up bound of surface moving acceleration (ax,ay)
ddxy_s_min = [-1.15;-1.15];      % minimum low bound of surface moving acceleration (ax,ay)
dT = 0.01;                            % simulation prediction time step
T_s = 2;                               % simulation time
from t=0.28s to end(t=2s)         % period of surface moving acceleration with random acc
time_stop = 0.81s                % MPC horizon beginning moment


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
a = -0.65;
b = 0.65;
random = a + (b-a)*rand(2,200);

if i >= 30
        ddxy_s = random(:,i-1);
        if abs(ddxy_s(1) - last_acc(1)) > 0.5
            ddxy_s(1) = last_acc(1) + sign((ddxy_s(1) - last_acc(1)))*0.5;
        end

        if abs(ddxy_s(2) - last_acc(2)) > 0.5
            ddxy_s(2) = last_acc(2) + sign((ddxy_s(2) - last_acc(2)))*0.5;
        end
        last_acc = ddxy_s;
    else
        ddxy_s = [0;0];
    end

