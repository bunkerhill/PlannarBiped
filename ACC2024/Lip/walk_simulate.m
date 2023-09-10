clc
clear
close all
%% set up
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')

L=0.26;
g=9.8;
ddxy_s_max = [0.05;0];
ddxy_s_min = [-0.05;0];

% choose a MPC controller
MPC_controller = intriMPC(g,L,ddxy_s_max,ddxy_s_min);

% figure
% plot(X_min_next,Y_min_next)
% axis equal
% hold on
% plot(X_max_next,Y_max_next)
% figure
% plot(X_min_next)
% hold on
% plot(X_max_next)
% figure
% plot(Y_min_next)
% hold on
% plot(Y_max_next)
% MPC_controller = contiMPC(g,L,ddxy_s_max,ddxy_s_min);

% time step
dT = 0.005;
%% simulation

x0 = [0;0;0;0];
T=0:dT:3;
x_tank = zeros(4,length(T));
x_tank(:,1) = x0; 
x_z = [0;0];
u_tank = [0;0];

% moving surface
ddxy_s = [0;0];
dxy_s = [0;0];
xy_s = [0;0];
xy_s_tank = [0;0];

for i=2:length(T)
    i
    current_T = T(i-1);
    if i > 100 && i < 110
        ddxy_s = [-0.01;0];
    else
        ddxy_s = [0;0];
    end
    x_z_dot = MPC_controller.MPC(x_tank(:,i-1),x_z,ddxy_s,current_T);
    x_z = x_z + dT * x_z_dot;
    u_tank = [u_tank, x_z];
    x_tank(:,i) = lip_dynamics(x_tank(:,i-1),x_z,ddxy_s,dT,L,g);

    dxy_s = dxy_s + ddxy_s * dT;
    xy_s = xy_s + dxy_s * dT;
    xy_s_tank = [xy_s_tank,xy_s];
end

%% plot result
figure
plot(T,x_tank(2,:))
hold on
plot(T,x_tank(4,:))

figure
plot(T,xy_s_tank(1,:))
title('moving surface trajectory')
xlabel('t (s)') 
ylabel('x (m)') 
% legend({'foot position','COM'},'Location','southwest')

figure
i = 600;
plot(x_tank(1,1:i))
hold on
plot(u_tank(1,1:i))

figure
i = 600;
plot(u_tank(1,1:i),u_tank(2,1:i))
axis equal 
%%
figure
[X_min_next,X_max_next] = MPC_controller.ZMP_rangex_plot(3);
[Y_min_next,Y_max_next] = MPC_controller.ZMP_rangey_plot(3);
for i=1:round(length(X_min_next)/2)
    rectangle('Position',[X_min_next(i),Y_min_next(i),X_max_next(i)-X_min_next(i),Y_max_next(i)-Y_min_next(i)],'FaceColor','c')
    hold on
end

i = length(X_min_next);
plot(u_tank(1,1:i),u_tank(2,1:i))

plot(x_tank(1,1:i),x_tank(3,1:i))

% plot(X_min_next,Y_min_next)
% plot(X_max_next,Y_max_next)


title('COM trajectory and foot position trajectory')
xlabel('x (m)') 
ylabel('y (m)') 
legend({'foot position','COM'},'Location','southwest')
axis equal 

%% dynamic equation
function X_next = lip_dynamics(X,u,ddxy_s,dT,L,g)

A1 = [1 dT;
     g/L*dT 1];
 
B1 = [0;
     -dT*g/L];

C1 = [0;
     -ddxy_s(1)];
C2 = [0;
     -ddxy_s(2)];

A = blkdiag(A1,A1);
B = blkdiag(B1,B1);
C = [C1;
     C2];
X_next = A*X + B*u + C;

end

% function X_next = lipode(X,u,ddxy_s,dT,L,g)
% 
% A1 = [1 dT;
%      g/L*dT 1];
%  
% B1 = [0;
%      -dT*g/L];
% 
% C1 = [0;
%      -ddxy_s(1)];
% C2 = [0;
%      -ddxy_s(2)];
% 
% A = blkdiag(A1,A1);
% B = blkdiag(B1,B1);
% C = [C1;
%      C2];
% X_next = A*X + B*u + C;
% 
% end