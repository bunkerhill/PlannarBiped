function [zmp] = LIP_MPC(uin)
% LIP_MPC: MPC using linear inverted pendulum model (including intrinsic stable MPC, contingency MPC)

%% MPC Parameters
global i_MPC_var i_gait currentTime dt_MPC_vec gait x_traj_IC Contact_Jacobian Rotm_foot addArm
k = i_MPC_var; % current stage
horizon = 10; % prediction horizons
g = 9.81; % gravity
ltoe = 0.09; % distance from ankle to toe
lheel = -0.06; % distance from ankle to heel
lhalfwidth = 0.01; % half width of foot

xdes=uin(1:12); % desired states [eul, p, omega, v]'
x=uin(13:24); % current states [eul, p, omega, v]'
q=uin(25:34); % joint angles [q_L, q_R]'
foot=uin(35:46); % contact position and velocity [p_R, v_R, p_L, v_L]'
rightFoot = foot(1:3);
leftFoot = foot(7:9);

% Linear inverted pendulum model. x position, velocity, y position velocity
xLIP = [x(4); x(10); x(5); x(11)];
comHeight=0.525;
omega = sqrt(g/comHeight);
xi=[xLIP(1)+xLIP(2)/omega; xLIP(3)+xLIP(4)/omega];
stepDuration=0.2;
averageSpeed=0.4;
stepWidth=0.2;

Nsteps=5;
footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
currentStanceFootID = 1 - i_gait;
currentStanceFootPosition = zeros(2,1);
if currentStanceFootID == 0
    % left foot is stance foot;
    currentStanceFootPosition = leftFoot(1:2);
else
    currentStanceFootPosition = rightFoot(1:2);
end
footPlanner=footPlanner.findOptimalFootPlacement(Nsteps, xi,currentStanceFootID,currentStanceFootPosition,currentTime);
footHalfLength=0.06;
footHalfWidth=0.01;
zmpController = intrinsicMPC(comHeight, footHalfLength, footHalfWidth);
currentZMP=currentStanceFootPosition;
zmpController = zmpController.MPC(xi, currentZMP, currentTime, footPlanner.stanceFootConstraint);
zmpController.drawZMPPreviewAndConstraint()
zmp=zmpController.getOptimalZMP();

% m = 5.75 + 2*(0.835+0.764+1.613+0.12+0.08); % mass (inclulded body, hips, and thighs)
% 
% Fx = m*g/comHeight * (xLIP(1)-zmp(1));
% Fy = m*g/comHeight * (xLIP(3)-zmp(2));
% Fz = m*g;
% My = Fz*(zmp(1)-currentStanceFootPosition(1));
% 
% if currentStanceFootID ==0
%     % left foot stance
%     GRF = [zeros(3,1); [Fx; Fy; Fz];zeros(3,1); [0;My;0]];
% else
%     GRF = [[Fx; Fy; Fz]; zeros(3,1); [0; My; 0]; zeros(3,1)];
% end
% GRF
end
