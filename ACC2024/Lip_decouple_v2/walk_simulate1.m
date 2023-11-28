% clc
% clear
% close all
%% set up
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')

L=0.26;
g=9.8;
ddxy_s_max = [1.15;1.15];
ddxy_s_min = [-1.15;-1.15];

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

up_u = [];
low_u = [];

last_acc = [0,0];

for i=2:length(T)
    fprintf("%d\n",i);
    current_T = T(i-1);
    if i >= 30
        ddxy_s = random(:,i-1); % 0.56  0.58
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

    if i == 93
        X_z_dot = MPC_controller2.MPC_horizon(x_tank(:,i-1),x_z,ddxy_s,current_T);

        x_z_up = x_z;
        for j = 1:(length(X_z_dot)/2)
            x_z_up = x_z_up + dT * X_z_dot(j,:)';
            up_u = [up_u,x_z_up];
        end
        x_z_low = x_z;
        for j = 1:(length(X_z_dot)/2)
            x_z_low = x_z_low + dT * X_z_dot(j+(length(X_z_dot)/2),:)';
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
[X_min_next,X_max_next] = MPC_controller2.ZMP_rangex(current_T);
[Y_min_next,Y_max_next] = MPC_controller2.ZMP_rangey(current_T);

figure


plot(up_u(1,:))
hold on
plot(low_u(1,:))
plot(X_min_next)
plot(X_max_next)
title('Contingency MPC in x direction')
xlabel('horizon (dT=0.01s)') 
ylabel('position (m)') 
legend({'normal_COM','up_COM','low_COM','ZMP_lowConstraint','ZMP_upConstraint'})

figure
plot(up_u(2,:))
hold on
plot(low_u(2,:))
plot(Y_min_next)
plot(Y_max_next)
title('Contingency MPC in y direction')
xlabel('horizon (dT=0.01s)') 
ylabel('position (m)') 
legend({'normal_COM','up_COM','low_COM','ZMP_lowConstraint','ZMP_upConstraint'})
%%
figure
[X_min_next,X_max_next] = MPC_controller2.ZMP_rangex_plot(T_s);
[Y_min_next,Y_max_next] = MPC_controller2.ZMP_rangey_plot(T_s);

for i=1:length(X_min_next)
    rectangle('Position',[X_min_next(i),Y_min_next(i),X_max_next(i)-X_min_next(i),Y_max_next(i)-Y_min_next(i)])
    hold on
end

% connect
u_tank = [u_tank, up_u(:,1)];

plot(u_tank(1,:),u_tank(2,:))


plot(up_u(1,:),up_u(2,:))
plot(low_u(1,:),low_u(2,:))


% plot(x_tank(1,1:i),x_tank(3,1:i))

% plot(X_min_next,Y_min_next)
% plot(X_max_next,Y_max_next)


title('foot position trajectory')
xlabel('x (m)') 
ylabel('y (m)') 
legend({'exist_COM','up_COM','low_COM'},'Location','southwest')
axis equal 


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