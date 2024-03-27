%% Contingency Model Predictive Control 

function u = IMPC_direct_Controller(uin)
tic
%% MPC Parameters
global i_MPC_var dt_MPC_vec gait x_traj_IC I_error Contact_Jacobian Rotm_foot addArm last_u MPC_controller x_z xy_com xy_com_act footprint xy_com_tank i_gait u_zmp desire_traj global_t
global u_zmp_tank x_z_tank ddxyz_com_tank p_xy_tank fx_end_R fx_end_L fy_end_R fy_end_L last_point up_u low_u moving_tank
global X_min X_max Y_min Y_max moving_xy stance_leg zmpController footPlanner com_x com_dx next_footHold footHoldVector
k = i_MPC_var; % current horizon
g = 9.81; % gravity

%% Making definition consistent with MPC
% input definition
%%%%%% All the variables below are in world frame %%%%%%
xdes=uin(1:12); % desired states [eul, p, omega, v]'
x=uin(13:24); % current states [eul, p, omega, v]'
q=uin(25:34); % joint angles [q_L, q_R]'
foot=uin(35:46); % contact position and velocity [p_L, v_L, p_R, v_R]'

eul = x(1:3);
eul_des = xdes(1:3);
R = eul2rotm(flip(eul'));

%% Assigning desired trajectory for CoM and Foot locations

%%%%%% All the variables below are in world frame %%%%%%
RPY=x(1:3); % Roll Pitch Yaw
x_act=x(4:6); % CoM position (x,y,z)
w_act=x(7:9); % Angular velocity (x,y,z)
v_act=x(10:12); % CoM velocity (x,y,z)
dT = 0.008; % control period (every 8ms run this code once)

% contingency MPC, only care about x-y plane motion
L = 0.525; % the height of COM

% surface position
xy_s = [moving_xy(1);moving_xy(4)];
dxy_s = [moving_xy(2);moving_xy(5)];
xy_s_com = [moving_xy(1);moving_xy(2);moving_xy(4);moving_xy(5)];
% ddxy_s = [0;0]; % suppose the ground surface is not moving
ddxy_s = [moving_xy(3);moving_xy(6)];

global_t

% for the beginning of stand on two feet, xy_com is the same as initial actual com
if global_t==0
    xy_com=[x(4);x(10);x(5);x(11)];
    u_zmp = (foot(1:2)+foot(7:8))/2;
end

% use real robot states
xy_com=[x(4);x(10);x(5);x(11)]-[moving_xy(1);moving_xy(2);moving_xy(4);moving_xy(5)];

%%% draw foot placement
% if rem(k,5) ~= last_point && last_point == 0
%     footPlanner.drawOptimalFootPlacement();
% end
% last_point = rem(k,5);

% get important data(predicted zmp and actual zmp)
if (i_gait==0) % R stance
    x_z = foot(1:2)-xy_s;
    current_zmp_x = foot(1)-xy_s(1);
    current_zmp_y = foot(2)-xy_s(2);
    % xy_com=[x(4);x(10);x(5);x(11)];
else
    x_z = foot(7:8)-xy_s;
    current_zmp_x = foot(7)-xy_s(1);
    current_zmp_y = foot(8)-xy_s(2);
end

x_z_tank = [x_z_tank x_z];
moving_tank = [moving_tank moving_xy];
stance_leg = [stance_leg i_gait];
footHoldVector = [footHoldVector next_footHold];

comHeight=0.525;
g=9.8;%m/s^2
omega=sqrt(g/comHeight);
xi = com_x + com_dx/omega;
currentZMP = [current_zmp_x;current_zmp_y];
currentTime = global_t;
zmpController = zmpController.MPC(xi, currentZMP, currentTime, footPlanner.stanceFootConstraint);
u_zmp = zmpController.getOptimalZMP();
% method 1
ddxy_com = lip_dynamics(xy_com([1,3]),u_zmp,ddxy_s,L,g);% use continuous lip model to calculate COM acceleration
ddxyz_com = [ddxy_com;0];
% method 2
xy_com = lip_dynamics_discrete(xy_com,u_zmp,ddxy_s,dT,L,g);

% print
xy_com_record = [xy_com(1);xy_com(3);xy_com(2);xy_com(4);ddxy_com];
ddxyz_com_tank = [ddxyz_com_tank xy_com_record];
u_zmp_tank = [u_zmp_tank u_zmp];

% ground reaction force optimization, only optimize to maintain z direction position and body orientation
kp_p = diag([250 250 300]); % kp weight for COM position (x,y,z) respectively
kd_p = diag([40 40 50]); % kd weight for COM velocity (x,y,z) respectively
kp_w = diag([1300 1300 1300]); % kp weight for COM axis angle (angx,angy,angz) respectively
kd_w = diag([100 100 100]); % kd weight for COM angular velocity (wx,wy,wz) respectively
x_traj = Calc_x_traj(xdes,x,dT)'; % get desired COM trajectory (position and axis angle will not be directly equal to user command)
w_com = x_traj(1:3); %  COM axis angle
xyz_com = x_traj(4:6); % COM position
dw_com = x_traj(7:9); % COM angular velociy
dxyz_com = x_traj(10:12); % COM velocity

% modify
xyz_com(1:2) = xy_com([1,3]);
dxyz_com(1:2) = xy_com([2,4]);


% For z direction, calculate desired COM linear acceleration with PD; 
% For y and x directions, directly use acceleration from contingency MPC's output
p_cddot = kp_p*(xyz_com-(x_act-[xy_s;0])) + kd_p*(dxyz_com-(v_act-[dxy_s;0])) + ddxyz_com; 

% Calculate desired COM angular acceleration with PID
R1 = eul2rotm([w_com(3),w_com(2),w_com(1)], 'ZYX');% rotation matrix of desired COM axis angle
R2 = eul2rotm([RPY(3),RPY(2),RPY(1)], 'ZYX');% rotation matrix of actual COM axis angle
R_error = R1 * R2'; % get the error of two coordinates
axis_angle = rotm2axang(R_error); % get axis angle from actual angle to desired angle
rpy_error = axis_angle(1:3)'*axis_angle(4); % get rotate angle from actual angle to desired angle
I_error = I_error + rpy_error; % integrate angle error
% I_error = [0;0;0];
w_cddot = kp_w*rpy_error + kd_w*(dw_com-w_act) + 1.5 * I_error; % kI = 1.5

foot_traj =[foot(1:3);foot(7:9)]; % assume foot under hip
desire_traj = [desire_traj x_traj]; % in order to plot desired trajectory
% desire_traj = [desire_traj rpy_error];

%% Robot simplified dynamics physical properties 
mu = 0.5; %friction coefficient
if addArm
    m = 16.4; % included arms for humanoid 
    Ib = diag([0.932, 0.9420, 0.0711]); % SRBD MoI (included body, arms, hips, and thighs)
else
    m = 5.75 + 2*(0.835+0.764+1.613+0.12+0.08); % mass (inclulded body, hips, and thighs)
    Ib = diag([0.5413, 0.5200, 0.0691]); % SRBD MoI (included body, hips, and thighs)
end
Fmax = 500; Fmin = 0; % force limits

%% Forward Kinematics (for joint torque constraints)
RR=reshape(R,[9,1]);
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')]; % torque=contact_mapping*u

%% State-space A & B matrices  (dynamics are done in world frame)
% foot rotation wrt. world frame
R_foot=Rotm_foot(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
R_foot_R=R_foot(1:3,:);
R_foot_L=R_foot(4:6,:);

I = R*Ib*R'; % MoI in world frame

% continuous A & b:
A=[eye(3), eye(3), zeros(3),zeros(3);
    skew(-x(4:6,1) + foot_traj(1:3,1)), skew(-x(4:6,1) + foot_traj(4:6,1)), eye(3), eye(3)];
b = [m*(p_cddot+[0;0;g]);
    I*w_cddot];
%% MPC Weights: (tune these according to your task)
% state tracking objective:
L = [100 0 0 0 0 0;
     0 100 0 0 0 0;
     0 0 130 0 0 0;
     0 0 0 1000 0 0;
     0 0 0 0 600 0;
     0 0 0 0 0 600];
W = 0.001*eye(12);
M = 1*eye(12);  
% L = eye(6);
% control input minimization:
% QP math:
Hd = 2*(A'*L*A+W+M);
fd = -2*(A'*L*b + M*last_u);  

%% QP Constraints:
% please refer to the quadprog constraint format: lbA <= A*u <= ubA
bigNum = 1e6;
smallNum = -bigNum;
Ac = DM(numCons,numVar); 
% Ac = zeros(numCons,numVar);
%friction constraint:
A_mu = [1,0,-mu,zeros(1,9);
    0,1,-mu,zeros(1,9); 
    1,0,mu,zeros(1,9); 
    0,1,mu,zeros(1,9);
    zeros(1,3),1,0,-mu,zeros(1,6);
    zeros(1,3),0,1,-mu,zeros(1,6); 
    zeros(1,3),1,0,mu,zeros(1,6); 
    zeros(1,3),0,1,mu,zeros(1,6)];

lba_mu = [smallNum;smallNum;0;0; smallNum;smallNum;0;0];
uba_mu = [0;0;bigNum;bigNum; 0;0;bigNum;bigNum];

%force limit constraint:
A_f = [0,0,1,zeros(1,9); zeros(1,3),0,0,1,zeros(1,6)];
lba_force = [Fmin; Fmin]; 
uba_force = [Fmax; Fmax];

% Line foot constraints :
% ref: Ding, Yanran, et al. "Orientation-Aware Model Predictive Control 
% with Footstep Adaptation for Dynamic Humanoid Walking." 2022 IEEE-RAS 
% 21st International Conference on Humanoid Robots (Humanoids). IEEE, 2022.

lt = 0.09-0.01; lh = 0.06-0.02;% line foot lengths
A_LF1=[-lh*[0,0,1]*R_foot_R',zeros(1,3),[0,1,0]*R_foot_R',zeros(1,3);
    -lt*[0,0,1]*R_foot_R',zeros(1,3),-[0,1,0]*R_foot_R',zeros(1,3);
    zeros(1,3),-lh*[0,0,1]*R_foot_L',zeros(1,3),[0,1,0]*R_foot_L';
    zeros(1,3),-lt*[0,0,1]*R_foot_L',zeros(1,3),-[0,1,0]*R_foot_L'];
A_LF2 = 1*[ [0, lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, -lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, 1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, 1]*R_foot_L';
    [0, -lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, -1]*R_foot_L'];
uba_LF = zeros(12,1); 
lba_LF = ones(12,1)*smallNum;
A_LF = [A_LF1;A_LF2]; 
% motor torque limit
A_tau = contact_mapping;
% gait are implemented here to zero out the swing leg control inputs
if gait == 1
    gaitm = gaitSchedule(i_gait);
elseif gait == 0
    gaitm = ones(10,1);
end
uba_tau = [33.5;33.5;50;50;33.5;33.5;33.5;50;50;33.5].*gaitm;
lba_tau = -uba_tau.*gaitm;

% foot moment Mx=0: 
Moment_selection=[1,0,0];
A_M = [zeros(1,3),zeros(1,3),Moment_selection*R_foot_R',zeros(1,3);
    zeros(1,3),zeros(1,3),zeros(1,3),Moment_selection*R_foot_L'];

uba_M = zeros(2,1);
lba_M = uba_M;

% Constraint Aggregation:
Ac(1:8,:) = A_mu; % row 1-80
Ac(9:10,:) = A_f; % row 81-100
Ac(11:22,:) = A_LF; % row 101-220
Ac(23:32,:) = A_tau; % row 221-320
Ac(33:34,:) = A_M; % row 321-340

lba = [lba_mu; lba_force; lba_LF; lba_tau; lba_M];
uba = [uba_mu; uba_force; uba_LF; uba_tau; uba_M];

%% quadprog
% A = [Ac;-Ac];
% b = [uba;-lba];
% 
% options = optimset('Algorithm','interior-point-convex','Display','off');
% [u,~,exitflag,~] = quadprog(Hd,fd,A,b,[],[],[],[],[],options);
% if exitflag == -2
%    fprintf("---SOLUTION NOT FOUND---");
% end

%% QPOASES setup:
Hsize = size(Hd);
fsize = size(fd);
H = DM(Hsize);
H = DM(Hd);
g = DM(fsize);
g = DM(fd);

qp = struct;
qp.h = H.sparsity();
qp.a = Ac.sparsity();
% qp.setOptions( setToReliable() );
% print_options()
opts = struct('enableEqualities', 1, 'printLevel', 'low',...
    'enableFullLITests',0);
S = conic('S','qpoases',qp,opts);
% disp(S)
r = S('h', H, 'g', g, 'a', Ac, 'lba',lba, 'uba', uba);

% solve!:
% tic
x_opt = r.x;
% disp(x_opt(1:12));
% disp(' ');
% disp(['MPC Time Step:',num2str(k)]);
% disp('QPOASES-MPC Solve Time:');
% toc

GRFM = full(x_opt);
GRFM = GRFM(1:12);
% u=-contact_mapping*GRFM(1:12); 
u=[GRFM(1:12)];

% disp('QPOASES-MPC Function Total Time:');
% toc
last_u = u;


end


%% functions %%
function x_traj = Calc_x_traj(xdes,x,dt)

for j = 1:6
     if xdes(6+j) == 0 % if velocity is zero, then position use desired position
         x_traj(j) = xdes(j) + xdes(6+j)*dt;
     else % if velocity is not zero, then position use current position
         x_traj(j) = x(j) + xdes(6+j)*dt;
     end
     x_traj(6+j) = xdes(6+j);
end

end

function gaitm = gaitSchedule(i)

% walking 
Rm = [0;0;0;0;0; 1;1;1;1;1];
Lm = [1;1;1;1;1; 0;0;0;0;0];

if(i==0) % R stance
    gaitm = Lm;
else
    gaitm = Rm;
end

end

function A= skew(v)
 A=[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ]; 
end

function ddXY = lip_dynamics(X,u,ddxy_s,L,g)

ddXY = g/L*(X-u)-ddxy_s;

end

function X_next = lip_dynamics_discrete(X,u,ddxy_s,dT,L,g)

A1 = [1 dT;
     g/L*dT 1];
 
B1 = [0;
     -dT*g/L];

C1 = [0;
     -ddxy_s(1)*dT];
C2 = [0;
     -ddxy_s(2)*dT];

A = blkdiag(A1,A1);
B = blkdiag(B1,B1);
C = [C1;
     C2];
X_next = A*X + B*u + C;

end