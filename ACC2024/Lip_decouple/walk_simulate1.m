clc
clear
close all
%% set up
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')

L=0.26;
g=9.8;
ddxy_s_max = [0.8;0];
ddxy_s_min = [-0.8;0];

% choose a MPC controller
% MPC_controller1 = intriMPC(g,L);
MPC_controller2 = contiMPC(g,L,ddxy_s_max,ddxy_s_min);

% time step
dT = 0.01;

% simulation time
T_s = 2;
%% simulation

x0 = [0;0;0;0];
T=0:dT:T_s;
x_tank = zeros(4,length(T));
x_tank(:,1) = x0; 
x_z = [0;0];
u_tank = [0;0];

% moving surface
ddxy_s = [0;0];
ddxy_s_tank = [0;0];
dxy_s = [0;0];
dxy_s_tank = [0;0];
xy_s = [0;0];
xy_s_tank = [0;0];

normal_u = [];
up_u = [];
low_u = [];

for i=2:length(T)
    fprintf("%d\n",i);
    current_T = T(i-1);
    if i >= 30 && i <= 130
        ddxy_s = [0.57;0]; % 0.56  0.58
    else
        ddxy_s = [0;0];
    end

    if i == 46
        X_z_dot = MPC_controller2.MPC_horizon(x_tank(:,i-1),x_z,ddxy_s,current_T);
        x_z_normal = x_z;
        for j = 1:(length(X_z_dot)/3)
            x_z_normal = x_z_normal + dT * X_z_dot(j,:)';
            normal_u = [normal_u,x_z_normal];
        end
        x_z_up = x_z;
        for j = 1:(length(X_z_dot)/3)
            x_z_up = x_z_up + dT * X_z_dot(j+(length(X_z_dot)/3),:)';
            up_u = [up_u,x_z_up];
        end
        x_z_low = x_z;
        for j = 1:(length(X_z_dot)/3)
            x_z_low = x_z_low + dT * X_z_dot(j+2*(length(X_z_dot)/3),:)';
            low_u = [low_u,x_z_low];
        end

        break
    end


    x_z_dot = MPC_controller2.MPC(x_tank(:,i-1),x_z,ddxy_s,current_T);
    x_z = x_z + dT * x_z_dot;
    u_tank = [u_tank, x_z];
    x_tank(:,i) = lip_dynamics(x_tank(:,i-1),x_z,ddxy_s,dT,L,g);

    dxy_s = dxy_s + ddxy_s * dT;
    xy_s = xy_s + dxy_s * dT;

    ddxy_s_tank = [ddxy_s_tank,ddxy_s];
    dxy_s_tank = [dxy_s_tank,dxy_s];
    xy_s_tank = [xy_s_tank,xy_s];

    
end

%% plot result
% [X_min_next,X_max_next] = MPC_controller2.get_ZMP_rangex(T_s+dT);
% [Y_min_next,Y_max_next] = MPC_controller2.get_ZMP_rangey(T_s+dT);
% figure
% plot(T,x_tank(2,:))
% hold on
% plot(T,x_tank(4,:))
% title('com velocity')
% xlabel('t (s)') 
% ylabel('v (m/s)') 
% legend({'v_x','v_y'})
% 
% figure
% plot(T,dxy_s_tank(1,:))
% title('moving surface x direction velocity')
% xlabel('t (s)') 
% ylabel('v (m/s)') 
% 
% figure
% plot(T,ddxy_s_tank(1,:))
% title('moving surface x direction acceleration')
% xlabel('t (s)') 
% ylabel('a (m/s^2)') 
% 
% figure
% plot(T,dxy_s_tank(2,:))
% title('moving surface y direction velocity')
% xlabel('t (s)') 
% ylabel('v (m/s)') 
% 
% figure
% plot(T,ddxy_s_tank(2,:))
% title('moving surface y direction acceleration')
% xlabel('t (s)') 
% ylabel('a (m/s^2)') 

% figure
% plot(T,x_tank(1,:))
% hold on
% plot(T,u_tank(1,:))
% plot(T+dT,X_min_next)
% plot(T+dT,X_max_next)
% title('com and zmp postion in x direction')
% xlabel('t (s)') 
% ylabel('position (m)') 
% legend({'COM','ZMP','ZMP_lowConstraint','ZMP_upConstraint'})
% 
% figure
% plot(T,x_tank(3,:))
% hold on
% plot(T,u_tank(2,:))
% plot(T+dT,Y_min_next)
% plot(T+dT,Y_max_next)
% title('com, zmp and zmp constraint postion in y direction')
% xlabel('t (s)') 
% ylabel('position (m)') 
% legend({'COM','ZMP','ZMP_lowConstraint','ZMP_upConstraint'})
% 
% figure
% i = 200;
% plot(u_tank(1,1:i),u_tank(2,1:i))
% axis equal 
%%
figure
[X_min_next,X_max_next] = MPC_controller2.ZMP_rangex_plot(T_s);
[Y_min_next,Y_max_next] = MPC_controller2.ZMP_rangey_plot(T_s);

for i=1:length(X_min_next)
    rectangle('Position',[X_min_next(i),Y_min_next(i),X_max_next(i)-X_min_next(i),Y_max_next(i)-Y_min_next(i)])
    hold on
end

% connect
u_tank = [u_tank, normal_u(:,1)];

plot(u_tank(1,:),u_tank(2,:))
plot(normal_u(1,:),normal_u(2,:))

plot(up_u(1,:),up_u(2,:))
plot(low_u(1,:),low_u(2,:))


% plot(x_tank(1,1:i),x_tank(3,1:i))

% plot(X_min_next,Y_min_next)
% plot(X_max_next,Y_max_next)


title('foot position trajectory')
xlabel('x (m)') 
ylabel('y (m)') 
legend({'exist_COM','normal_COM','up_COM','low_COM'},'Location','southwest')
axis equal 

%%
figure
plot(normal_u(1,:))
hold on
plot(up_u(1,:))
plot(low_u(1,:))
title('com, zmp and zmp constraint postion in y direction')
xlabel('t (s)') 
ylabel('position (m)') 
legend({'COM','ZMP','ZMP_lowConstraint','ZMP_upConstraint'})

figure
plot(normal_u(2,:))
hold on
plot(up_u(2,:))
plot(low_u(2,:))
title('com, zmp and zmp constraint postion in y direction')
xlabel('t (s)') 
ylabel('position (m)') 
legend({'COM','ZMP','ZMP_lowConstraint','ZMP_upConstraint'})

%% dynamic equation
function X_next = lip_dynamics(X,u,ddxy_s,dT,L,g)

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