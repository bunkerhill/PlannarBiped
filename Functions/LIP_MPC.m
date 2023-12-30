function u = LIP_MPC(uin)
% LIP_MPC: MPC using linear inverted pendulum model (including intrinsic stable MPC, contingency MPC)

%% MPC Parameters
global i_MPC_var dt_MPC_vec gait x_traj_IC Contact_Jacobian Rotm_foot addArm
k = i_MPC_var; % current stage
horizon = 10; % prediction horizons
g = 9.81; % gravity
schedule = stanceSchedule(k);
ltoe = 0.09; % distance from ankle to toe
lheel = -0.06; % distance from ankle to heel
lhalfwidth = 0.01; % half width of foot

xdes=uin(1:12); % desired states [eul, p, omega, v]'
x=uin(13:24); % current states [eul, p, omega, v]'
q=uin(25:34); % joint angles [q_L, q_R]'
foot=uin(35:46); % contact position and velocity [p_R, v_R, p_L, v_L]'
rightFoot = foot(1:3);
leftFoot = foot(7:9);

zmpRange = ZMPRange(k, rightFoot, leftFoot, schedule, ltoe, lheel, lhalfwidth);


% Linear inverted pendulum model. x position, velocity, y position velocity
xLIP = [x(4); x(10); x(5); x(11)];



u=0;
end


function [schedule] = stanceSchedule(currentTimeIdx)
% stance schedule for the MPC prediction horizon: 2*horizon matrix 
% schedule(1,:) right foot, stance 0 swing 1 
% schedule(2,:) left foot, stance 0 swing 1
% schedule(3,:) if stance foot changes 1, otherwise 0
h = 10;
k = rem(currentTimeIdx, h);
if k==0
    k=10;
end

% The simulation starts with right foot swing, left foot stance.
rfoot = [1;1;1;1;1; 0;0;0;0;0];
lfoot = [0;0;0;0;0; 1;1;1;1;1];

j = k:k+h-1;
j(j>h) = j(j>h)-h;

schedule = zeros(3, h);
schedule(1,:) = rfoot(j);
schedule(2,:) = lfoot(j);

stanceFootChangeIdx = [];
for i=2:h
    if schedule(1,i) ~= schedule(1, i-1)
        schedule(3, i)=1;
        stanceFootChangeIdx = [stanceFootChangeIdx, i];
    end
end
end


function zmp_range = ZMPRange(currentTimeIdx, rightFoot, leftFoot, schedule, ltoe, lheel, lhalfwidth)
horizon = 10;
% x_max, x_min, y_max, y_min
zmp_range = zeros(4, horizon);
stepLength = 0.07;
stepWidth = 0.1;

currentStanceFootPos=[];
currentStanceFoot = 1; % 1 left foot; -1 right foot (lateral position).
if schedule(1,1) == 1
    % left foot stance
    currentStanceFootPos = leftFoot;
    currentStanceFoot = 1;
else
    % right foot stance
    currentStanceFootPos = rightFoot;
    currentStanceFoot = -1;
end

zmp_range(1,1) = currentStanceFootPos(1) + ltoe;
zmp_range(2,1) = currentStanceFootPos(1) + lheel;
zmp_range(3,1) = currentStanceFootPos(2) + lhalfwidth;
zmp_range(4,1) = currentStanceFootPos(2) - lhalfwidth;

num_step=0;
for i=2:horizon
    if schedule(3, i) == 0
        % stance foot not change
        zmp_range(:,i) = zmp_range(:,i-1);
    else
        % stance foot changes
        num_step = num_step+1;
        predictedStanceFootPos = [currentStanceFootPos(1) + num_step * stepLength;
                                currentStanceFoot*(-1)^num_step*stepWidth];
        zmp_range(1,i) = predictedStanceFootPos(1) + ltoe;
        zmp_range(2,i) = predictedStanceFootPos(1) + lheel;
        zmp_range(3,i) = predictedStanceFootPos(2) + lhalfwidth;
        zmp_range(4,i) = predictedStanceFootPos(2) - lhalfwidth;
    end
end
end


function optimal_zmp = optimalZMP(x_LIP, zmpRange)


end