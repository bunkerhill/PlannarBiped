clear;
%% HECTOR OPEN SOURCE SIMULATION SOFTWARE IN MATLAB/SIMULINK%%
% PLEASE READ LICENSE AGREEMENT BEFORE PROCEEDING

%% Run this script before simulation to load parameters

%% Pre-formulate functions
% addpath(genpath('casadi-windows-matlabR2016a-v3.5.1'))
import casadi.*
% generate global symbolic functions:
global Contact_Jacobian Rotm_foot MPC_controller x_z xy_com xy_com_act footprint xy_com_tank desire_traj last_u I_error u_zmp global_t u_zmp_tank x_z_tank moving_tank
global ddxyz_com_tank p_xy_tank fx_end_R fx_end_L fy_end_R fy_end_L last_point moving_xy up_u low_u
global X_min X_max Y_min Y_max stance_leg dx dy x y random count last_acc Ax Ay T_periodx T_periody
global footPlanner zmpController next_footHold Leftfoot Rightfoot com_x com_dx footHoldVector
[Contact_Jacobian,Rotm_foot]=Formulate_Contact_Jacobian;

%% adaptive foot placement planner
comHeight=0.525;% center of mass height
stepDuration=0.2;%sec
averageSpeed=0.5;%m/s
stepWidth=0.2;%m
Leftfoot = [0;0];
Rightfoot = [0;0];
com_x = [0;0];
com_dx = [0;0];
next_footHold = [0;0];
footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
%% intrinsic mpc
footHalfLength=0.06;
footHalfWidth=0.01;
zmpController = intrinsicMPC(comHeight, footHalfLength, footHalfWidth);
%%
% set rand
% a = -10;
% b = 10;
% randomx = a + (b-a)*rand(1,10000);
% randomy = a/2 + (b-a)/2*rand(1,10000);
% random = [randomx;randomy];

% Ax = 0.1;
% T_periodx = 0.6;
% Ay = 0.1;
% T_periody = 0.6;
Ax = 0.07;
T_periodx = 0.6;
Ay = 0.05;
T_periody = 0.8;

last_acc = [0;0];
count = 1;
L=0.525;
g=9.81;
ddxy_s_max = [15;15];
ddxy_s_min = [-15;-15];
% ddxy_s_max = [0;0];
% ddxy_s_min = [0;0];
MPC_controller = CMPC(g,L,ddxy_s_max,ddxy_s_min);
% MPC_controller = ACC_MPC(g,L);
xy_com = [0;0;0;0];
xy_com_tank = [0;0;0;0];
xy_com_act = [0;0;0;0];
footprint = [0;0];
desire_traj = [];
last_u = zeros(12,1);
I_error = 0;
u_zmp = [0;0];
u_zmp_tank = [];
x_z = [0;0];
x_z_tank = [];
stance_leg = [];
moving_tank = [];
ddxyz_com_tank = [];
footHoldVector = [];
p_xy_tank = [];
fx_end_R = 0; 
fx_end_L = 0;
fy_end_R = 0;
fy_end_L = 0;
last_point = 0;
moving_xy = [0;0;0;0;0;0];
up_u = [];
low_u = [];
dx = Ax*2*pi/T_periodx;
dy = Ay*2*pi/T_periody;
% dx = 0;
% dy = 0;
x = 0;
y = 0;
%% General (sim world physics)
world_damping = 1e-3;
world_rot_damping = 1e-3;
joint_stiffness = 0;
joint_damping = 0.001;
% elastic stiff ground:
contact_stiffness = 1e5;
contact_damping = 1e3; 
contact_point_radius = 0.001; % contact cloud visual
ankle_stiffness = 0.0;
ankle_damping = 0.0;
% elastic joint hard stop:
limit_stiffness = 1e3;
limit_damping = 1e2;
% ground friction (static/kinematic)
mu_s = 1.0;
mu_k = 1.0;
mu_vth = 0.001; % critical velocity for transition (m/s)

%% Controller global params
global p_foot_w p_foot_pre t_start gaitcycle dt_MPC N gait dt_MPC_vec ...
    acc_t current_height addArm

N = 5000; % place holder for total MPC prediction length
p_foot_w = zeros(6,1); % initial foot position
t_start = 0; % simulation start time

% MPC setup:
gaitcycle = 0.4; % gait period
h = 10; % horizon length
dt_MPC = gaitcycle/10; % MPC sample time  0.04s
p_foot_pre = zeros(6,N+10); % foot matrix initialization 

% Gait setup:
gait = 1; % Standing = 0; walking = 1; 

%% grid surface definition (if using uneven terrain)
% make sure to switch to "Mesh Grid Surface" in Hector model
% sine wave (example)
x_grid = -1:0.01:5;
y_grid = -1:0.01:1;
[X,Y] = meshgrid(x_grid,y_grid);
z_heights = [0.05*cos(2*pi*X)-0.05]';

%% contact cloud:
% foot contact approximated by point clouds:
npt = 5; % number of contact points per line
contact_cloud = [ [[linspace(-0.06,0.09,npt)]',ones(npt,1)*0, ones(npt,1)*0]; 
                    [[linspace(-0.06,0.09,npt)]',ones(npt,1)*0.01, ones(npt,1)*0]; 
                    [[linspace(-0.06,0.09,npt)]',ones(npt,1)*-0.01, ones(npt,1)*0]];
mark_distance=1;
mark_x=-5:mark_distance:5;
mark_y=-5:mark_distance:5;
mark_cloud_x=repmat(mark_x,length(mark_y),1);
mark_cloud_x=reshape(mark_cloud_x,[length(mark_x)*length(mark_y),1]);
mark_cloud_y=repmat(mark_y',length(mark_x),1);
mark_cloud=[mark_cloud_x,mark_cloud_y,zeros(length(mark_x)*length(mark_y),1)];

% scatter3(contact_cloud(:,1),contact_cloud(:,2),contact_cloud(:,3))
% axis equal 
%% vairable MPC dt (feature will be implemented soon)
% See function AssignMPCStage(t) %
global fixed_MPC_dt
fixed_MPC_dt = 1; % fixed dt = 1;
dt_MPC_vec = define_dt_MPC(dt_MPC); % vector defining MPC dts
acc_t(:,1) = zeros(length(dt_MPC_vec)+1,1);
for ith = 2:length(dt_MPC_vec)+1
    acc_t(ith,1) = sum(dt_MPC_vec(1:ith-1));
end

%% Initial Condition/Parameter of Robot State/Joints
body_x0 = 0;
body_y0 = 0;
body_z0 = 0.525; % m
current_height = body_z0;
body_v0 = [0 0 0];
body_R = [0 0 0]; % deg.
% hip1 (Rz joint)
hip1_q0 = 0;
hip1_dq0 = 0;
hip_max = 30;
hip_min = -30;
% hip2 (Rx joint)
hip2_q0_L = 0;
hip2_dq0_L = 0;
hip2_q0_R = 0;
hip2_dq0_R = 0;
hip2_max = 18;
hip2_min = -18;
% thigh (Ry joint)
thigh_q0 = 45;
thigh_dq0 = 0;
thigh_max = 120;
thigh_min = -120;
% calf (Ry joint)
calf_q0 = -90;
calf_dq0 = 0;
knee_max = -15;
knee_min = -160;
% toe/ankle (Ry joint)
toe_q0 = 45;
toe_dq0 = 0;
ankle_max = 75;
ankle_min = -75;

% arms:
%initiate addArm
addArm = 0;
% shoulder Roll(Rx joint):
shoulderx_q0 = 0;
shoulderx_min = -15;
shoulderx_max = 90;
% shoulder Pitch(Ry joint):
shouldery_q0 = 0;
shouldery_min = -90;
shouldery_max = 90;
% Elbow (Ry joint):
elbow_q0 = -90;
elbow_min = -150;
elbow_max = -10;

%% Fncs
function out = define_dt_MPC(vec) 
global dt_MPC fixed_MPC_dt
    out = []; % define your variable MPC dts here
    for i=1:length(vec)
        out = [out;vec(i)*ones(5,1)];
    end
    if boolean(fixed_MPC_dt)
        out = [out;dt_MPC*ones(5*(1000-i),1)]; % fixed dt_MPC vec
    end
end

