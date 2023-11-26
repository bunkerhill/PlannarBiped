%% min(AF-b)'*S*(AF-b)
% foot rotation wrt. world frame
R_foot=Rotm_foot(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
R_foot_R=R_foot(1:3,:);
R_foot_L=R_foot(4:6,:);

I = R*Ib*R'; % MoI in world frame

% continuous A & b:
A=[eye(3), eye(3), zeros(3),zeros(3);
    skew(-x_traj(4:6,1) + foot_traj(1:3,1)), skew(-x_traj(4:6,1) + foot_traj(4:6,1)), eye(3), eye(3)];
b = [m*(p_cddot+g);
    I*w_cddot];
% discretization:
% B{i}=Bc*dt_MPC_vec(i+k-1);
% A_hat{i}=eye(13)+Ac*dt_MPC_vec(i+k-1);