close all
%% set up
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')
T=0:0.008:4;
%% plot
figure
plot(T,u_zmp_tank(1,:))
hold on
plot(T,x_z_tank(1,:)-0.06)
plot(T,x_z_tank(1,:)+0.06)

title('foot placement(ZMP) in x direction')
xlabel('time (s)') 
ylabel('position (m)') 
legend({'desired zmp','actual zmp low','actual zmp high'})

figure
plot(T,u_zmp_tank(end,:))
hold on
plot(T,x_z_tank(end,:)-0.01)
plot(T,x_z_tank(end,:)+0.01)

title('foot placement(ZMP) in y direction')
xlabel('time (s)') 
ylabel('position (m)') 
legend({'desired zmp','actual zmp low','actual zmp high'})

figure
plot(x_z_tank(1,:),x_z_tank(end,:))
hold on
% plot(u_zmp_tank(1,:),u_zmp_tank(end,:))
% plot(out.xout(:,4),out.xout(:,5))
% plot(ddxyz_com_tank(1,:),ddxyz_com_tank(2,:))
plot(footHoldVector(1,:),footHoldVector(2,:),"*")
plot(footHoldVector(1,:),footHoldVector(2,:))
title('foot placement(ZMP) in x-y plane')
xlabel('x position (m)') 
ylabel('y position (m)') 
legend({'actual zmp','desired zmp','actual com','desired com'})
axis equal 

% footprint
figure

for i=1:length(x_z_tank(1,:))
    rectangle('Position',[x_z_tank(1,i)-0.06,x_z_tank(end,i)-0.01,0.12,0.02])
    hold on
end
plot(x_z_tank(1,:),x_z_tank(end,:))
% plot(u_zmp_tank(1,:),u_zmp_tank(end,:))
plot(out.xout(:,4)'-moving_tank(1,:),out.xout(:,5)'-moving_tank(4,:))
% plot(ddxyz_com_tank(1,:),ddxyz_com_tank(2,:))
title('foot placement(ZMP) in x-y plane')
xlabel('x position (m)') 
ylabel('y position (m)') 
legend({'actual zmp','desired zmp','actual com','desired com'})
axis equal 
%%
% figure
% for i=1:length(X_min)
%     rectangle('Position',[X_min(i),Y_min(i),X_max(i)-X_min(i),Y_max(i)-Y_min(i)])
%     hold on
% end
% plot(X_min,Y_min)
% plot(up_u(1,:),up_u(2,:))
% plot(low_u(1,:),low_u(2,:))
% 
% title('foot position trajectory')
% xlabel('x (m)') 
% ylabel('y (m)') 
% legend({'0','up_COM','low_COM'},'Location','southwest')
% axis equal 

%%
% figure
% 
% 
% plot(up_u(1,:))
% hold on
% plot(low_u(1,:))
% plot(X_min)
% plot(X_max)
% title('Contingency MPC in x direction')
% xlabel('horizon (dT=0.01s)') 
% ylabel('position (m)') 
% legend({'up_zmp','low_zmp','ZMP_lowConstraint','ZMP_upConstraint'})
% 
% figure
% plot(up_u(2,:))
% hold on
% plot(low_u(2,:))
% plot(Y_min)
% plot(Y_max)
% title('Contingency MPC in y direction')
% xlabel('horizon (dT=0.01s)') 
% ylabel('position (m)') 
% legend({'up_zmp','low_zmp','ZMP_lowConstraint','ZMP_upConstraint'})
%%
figure % velocity
plot(T,out.xout(:,10))
hold on
plot(T,out.xout(:,11))
plot(T,ddxyz_com_tank(3,:))
plot(T,ddxyz_com_tank(4,:))
title('com velocity')
xlabel('time (s)') 
ylabel('velocity (m/s)') 
legend({'actual x-com','actual y-com','desired x-com','desired y-com'})

figure % position
plot(T,out.xout(:,4))
hold on
plot(T,out.xout(:,5))
plot(T,ddxyz_com_tank(1,:))
plot(T,ddxyz_com_tank(2,:))
title('com position')
xlabel('time (s)') 
ylabel('position (m)') 
legend({'actual x-com','actual y-com','desired x-com','desired y-com'})
(66+69+660+25.3+472+44.66+36.48+116+23.9+128+57.11+125+29+295)/2;

%%
figure % velocity
plot(T,out.xout(:,10))
hold on
plot(T,out.xout(:,11))
title('com velocity')
xlabel('time (s)') 
ylabel('velocity (m/s)') 
legend({'actual x-com','actual y-com'})

figure % position
plot(T,out.xout(:,4),'-', 'LineWidth',1.5)
hold on
plot(T,out.xout(:,5),'--', 'LineWidth',1.5)
plot(T,out.xout(:,6),'-.', 'LineWidth',1.5)
xlabel('time (s)') 
ylabel('position (m)') 
legend({'actual x-com','actual y-com','actual z-com'})
set(gca,'fontsize',14)%%
%%
% footprint
% figure
% 
% for i=1:length(x_z_tank(1,1:160))
%     rectangle('Position',[x_z_tank(1,i)-0.06,x_z_tank(end,i)-0.01,0.12,0.02])
%     hold on
% end
% plot(x_z_tank(1,1:160),x_z_tank(end,1:160))
% % plot(u_zmp_tank(1,:),u_zmp_tank(end,:))
% plot(out.xout(:,4)'-moving_tank(1,:),out.xout(:,5)'-moving_tank(4,:))
% % plot(ddxyz_com_tank(1,:),ddxyz_com_tank(2,:))
% title('foot placement(ZMP) in x-y plane')
% xlabel('x position (m)') 
% ylabel('y position (m)') 
% legend({'actual zmp','actual com'})
% axis equal

%%
figure
plot(T,moving_tank(3,:))
hold on
plot(T,moving_tank(6,:),'--')
xlabel('time (s)') 
ylabel('Disturbance (m/s^2)') 
legend({'x-direction','y-direction'})

figure
plot(T,moving_tank(1,:))
hold on
plot(T,moving_tank(4,:),'--')
plot(T,moving_tank(2,:))
plot(T,moving_tank(5,:),'--')
xlabel('time (s)') 
ylabel('moving surface motion') 
legend({'x-position','y-position','x-velocity','y-velocity'})
%% 3D plot
figure

for i=1:length(x_z_tank(1,:))
    rectangle('Position',[x_z_tank(1,i)-0.06,x_z_tank(end,i)-0.01,0.12,0.02],'FaceColor',"r")
    hold on
end
% plot3(x_z_tank(1,:),x_z_tank(end,:),zeros(1,length(x_z_tank(end,:))),'.')
% hold on
plot3(out.xout(:,4)'-moving_tank(1,:),out.xout(:,5)'-moving_tank(4,:),out.xout(:,6)','.',"Color","r")
for i=1:length(x_z_tank(1,:))
    if(stance_leg(i)==1)
        line([x_z_tank(1,i) out.xout(i,4)'-moving_tank(1,i)],[x_z_tank(2,i) out.xout(i,5)'-moving_tank(4,i)],[0 out.xout(i,6)'],"Color","b")
    else
        line([x_z_tank(1,i) out.xout(i,4)'-moving_tank(1,i)],[x_z_tank(2,i) out.xout(i,5)'-moving_tank(4,i)],[0 out.xout(i,6)'],"Color","y")
    end
end

[X,Y] = meshgrid(-0.5:0.1:3.5,-0.5:0.1:0.5);
Z = 0*X;
surf(X,Y,Z)
colorbar

title('foot placement(ZMP)')
xlabel('x position (m)') 
ylabel('y position (m)') 
zlabel('z position (m)') 
legend({'actual zmp','actual com'})
axis equal