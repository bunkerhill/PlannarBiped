% function [i_MPC_var,dt_MPC] = AssignMPCStage(t)
function xy = AssignMPCStage(t)

% This function is used for variable MPC dt in adaptive frequency MPC %

%  gait: walking = 1; hopping = 2; running = 3

global acc_t dt_MPC_vec i_MPC_var dt_MPC i_gait gait global_t moving_xy dx dy x y count random
global last_acc Ax Ay T_periodx T_periody
global_t = t;
dt = 0.0005;

% sine curve

% x = Ax*sin((t - 0.2)*2*pi/T_periodx);
% dx = Ax*2*pi/T_periodx*cos((t - 0.2)*2*pi/T_periodx);

% y = Ay*sin((t - 0.2)*2*pi/T_periody);
% dy = Ay*2*pi/T_periody*cos((t - 0.2)*2*pi/T_periody);

% ddx = -Ax*2*pi/T_periodx*2*pi/T_periodx*sin((t - 0.2)*2*pi/T_periodx);
% ddy = -Ay*2*pi/T_periody*2*pi/T_periody*sin((t - 0.2)*2*pi/T_periody);

% random
% ddxy_s = random(:,count); % 0.56  0.58
% if abs(ddxy_s(1) - last_acc(1)) > 3
%     ddxy_s(1) = last_acc(1) + sign((ddxy_s(1) - last_acc(1)))*3;
% end
% 
% if abs(ddxy_s(2) - last_acc(2)) > 3
%     ddxy_s(2) = last_acc(2) + sign((ddxy_s(2) - last_acc(2)))*3;
% end
% last_acc = ddxy_s;
% count = count + 1;

% sin curve
ddx = -Ax*2*pi/T_periodx*2*pi/T_periodx*sin(t*2*pi/T_periodx);
ddy = -Ay*2*pi/T_periody*2*pi/T_periody*sin(t*2*pi/T_periody);

% ddx = ddx + ddxy_s(1);
% ddy = ddy + ddxy_s(2);

dx = dx + ddx*dt;
x = x + dx*dt;
dy = dy + ddy*dt;
y = y + dy*dt;
% moving_xy = [x;dx;ddx;y;dy;ddy];
moving_xy = [0;0;0;0;0;0];

idx = find(acc_t<t + 1e-3);  %Find indices of nonzero elements.
i_MPC_var = idx(end); % get current MPC stage index
dt_MPC = dt_MPC_vec(i_MPC_var);
idx_gait = rem(i_MPC_var,10); % get current gait index
if gait <= 2
    if 1 <= idx_gait && idx_gait <= 5
        i_gait = 1;
    else
        i_gait = 0;
    end
elseif  gait == 3
    if 1 <= idx_gait && idx_gait <= 4
        i_gait = 1;
    elseif 6 <= idx_gait && idx_gait <= 9
        i_gait = 0;
    else
        i_gait = 2;

    end
end

xy = moving_xy;

end