%% Contingency Model Predictive Control 

function u = Contingency_MPC_v2(uin)
tic
%% MPC Parameters
global i_MPC_var dt_MPC_vec gait x_traj_IC I_error Contact_Jacobian Rotm_foot addArm last_u MPC_controller x_z xy_com xy_com_act footprint xy_com_tank i_gait desire_traj
k = i_MPC_var; % current horizon
h = 10; % prediction horizons
g = 9.81; % gravity

%% QPOASES Parameters
import casadi.*
numVar = 12;
numCons = 34;
%% Making definition consistent with MPC
% input definition
xdes=uin(1:12); % desired states [eul, p, omega, v]'
x=uin(13:24); % current states [eul, p, omega, v]'
q=uin(25:34); % joint angles [q_L, q_R]'
foot=uin(35:46); % contact position and velocity [p_L, v_L, p_R, v_R]'

eul = x(1:3);
eul_des = xdes(1:3);
R = eul2rotm(flip(eul'));

% enforce "2*pi=0" relation for turning
% yaw_correction=0;
% while yaw_correction==0
%     if eul_des(3,1)-eul(3,1)>pi
%         eul(3,1)=eul(3,1)+2*pi;
%     elseif eul_des(3,1)-eul(3,1)<-pi
%         eul(3,1)=eul(3,1)-2*pi;
%     else
%         yaw_correction=1;
%     end
% end

%% Assigning desired trajectory for CoM and Foot locations

RPY=x(1:3); % Roll Pitch Yaw
x_act=x(4:6); % CoM position
w_act=x(7:9); % Angular velocity
v_act=x(10:12); % CoM velocity
dT = 0.008;

% contingency part
% L = 0.525;
% ddxy_s = [0;0];
% if (i_gait==0) % R stance
%     x_z = foot(1:2);
% else
%     x_z = foot(4:5);
% end
% u_zmp_dot = MPC_controller.MPC([x(4);x(10);x(5);x(11)],x_z,ddxy_s);
% MPC_controller = MPC_controller.updatetime(dT);
% u_zmp = x_z + dT * u_zmp_dot;
% ddxy_com = lip_dynamics([x(4);x(5)],u_zmp,ddxy_s,L,g);
% ddxyz_com = [ddxy_com;0];
% dxyz_com = v_act + ddxyz_com * dT;
% dxyz_com(3) = 0;
% xyz_com = x_act + dxyz_com * dT;
% xyz_com(3) = L;
% % ddw_com = [0;0;0];
% dw_com = [0;0;0];
% w_com = [0;0;0];
% kp_p = diag([100 100 300]);
% kd_p = diag([10 10 50]);
% kp_w = diag([1600 1600 0]);
% kd_w = diag([50 50 10]);
% p_cddot = kp_p*(xyz_com-x_act) + kd_p*(dxyz_com-v_act) + ddxyz_com;
% % w_cddot = kp_w*(w_com-RPY) + kd_w*(dw_com-w_act) + ddw_com;
% % foot_traj =[foot(1:3);foot(7:9)]; % assume foot under hip
% desire_traj = [desire_traj [w_com;xyz_com;dw_com;dxyz_com;ddxyz_com]];

% ordinary control part
kp_p = diag([150 150 300]);
kd_p = diag([50 50 50]);
kp_w = diag([1300 1300 1300]);
kd_w = diag([100 100 100]);
x_traj = Calc_x_traj(xdes,x,dT)'; % desired trajectory
w_com = x_traj(1:3);
xyz_com = x_traj(4:6);
dw_com = x_traj(7:9);
dxyz_com = x_traj(10:12);
p_cddot = kp_p*(xyz_com-x_act) + kd_p*(dxyz_com-v_act);

%%
if w_com(3) == RPY(3) & w_com(2) == RPY(2) & w_com(1) == RPY(1)
    rpy_error = [0;0;0];
else
    R1 = eul2rotm([w_com(3),w_com(2),w_com(1)], 'ZYX');
    R2 = eul2rotm([RPY(3),RPY(2),RPY(1)], 'ZYX');
    R_error = R1 * R2';
    
    % 使用logm函数计算对数映射
    log_R_error = logm(R_error);

    % 提取反对称矩阵部分
    skew_symmetric_matrix = (log_R_error - log_R_error') / 2;

    % 提取旋转轴和角度
    angle_error = norm([skew_symmetric_matrix(3,2); skew_symmetric_matrix(1,3); skew_symmetric_matrix(2,1)], 2);
    axis_error = [skew_symmetric_matrix(3,2); skew_symmetric_matrix(1,3); skew_symmetric_matrix(2,1)] / sin(angle_error);

    % 定义轴角表示法
    axis_angle = [axis_error(3) axis_error(2) axis_error(1) angle_error];  % [x, y, z, angle]
    axis_angle = rotm2axang(R_error);
    % axis_angle = rotm2axang(R_error);
    % 转换为欧拉角
    Rotm = axang2rotm(axis_angle);  % 'ZYX' 表示旋转顺序为 ZYX
    rpy_error = rotm2eul(Rotm, 'ZYX')';
    rpy_error = axis_error*angle_error;
    rpy_error = axis_angle(1:3)'*axis_angle(4);
end
%%
I_error = I_error + rpy_error
% I_error = [0;0;0];
w_cddot = kp_w*rpy_error + kd_w*(dw_com-w_act) + 1.5 * I_error;
foot_traj =[foot(1:3);foot(7:9)]; % assume foot under hip
desire_traj = [desire_traj x_traj];
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
    gaitm = gaitSchedule(i_gait, 1);
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
tic
x_opt = r.x;
% disp(x_opt(1:12));
disp(' ');
disp(['MPC Time Step:',num2str(k)]);
disp('QPOASES-MPC Solve Time:');
toc

GRFM = full(x_opt);
GRFM = GRFM(1:12);
% u=-contact_mapping*GRFM(1:12); 
u=[GRFM(1:12)];

disp('QPOASES-MPC Function Total Time:');
toc
last_u = u;


end


%% functions %%

function foot_traj = Calc_foot_traj_3Dwalking(xdes,x_traj,foot,h,k,R)
global dt_MPC_vec 
i_MPC_gait = rem(k,h);

foot_traj = zeros(6,h);
%foot prediction: walking gait
if 1 <= i_MPC_gait && i_MPC_gait <= 5 % stance sequence: R-L-R
    current_R = foot(1:3); % current right foot - anchor foot
    next_L = current_R + xdes(10)*dt_MPC_vec(k)*h/2;
    next_R = next_L + xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: right foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(4:6,i) = R*current_R;
    end
    %phase 2: next left foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(1:3,i) = R*next_L;
    end
    %phase 3: next right foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(4:6,i) = R*next_R;
        end
    end
else % stance sequence: L-R-L
    if i_MPC_gait == 0; i_MPC_gait = 10; end
    i_MPC_gait = i_MPC_gait - h/2;
    current_L = foot(1:3); % current left foot - anchor foot
    next_R = current_L + n*xdes(10)*dt_MPC_vec(k)*h/2;
    next_L = next_R + n*xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: left foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(1:3,i) = R*current_L;
    end
    %phase 2: next right foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(4:6,i) = R*next_R;
    end
    %phase 3: next left foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(1:3,i) = R*next_L;
        end
    end
end

end


function x_traj = Calc_x_traj(xdes,x,dt)

for j = 1:6
     if xdes(6+j) == 0
         x_traj(j) = xdes(j) + xdes(6+j)*dt;
     else
         x_traj(j) = x(j) + xdes(6+j)*dt;
     end
     x_traj(6+j) = xdes(6+j);
end

end

function gaitm = gaitSchedule(i, gait)

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