% footprint
num = 1;
x_z = x_z_tank(:,1:num:end);
u_zmp = u_zmp_tank(:,1:num:end);

figure

for i=1:length(x_z(1,:))-1
    rectangle('Position',[x_z(1,i)-0.06,x_z(end,i)-0.01,0.12,0.02],'LineWidth',1.5,'EdgeColor','r')
    % r = 0.04;
    % rectangle('Position',[x_z_tank(1,i)-r,x_z_tank(end,i)-r,2*r,2*r],'Curvature',[1 1],'EdgeColor','r')
    hold on
end
plot(x_z_tank(1,:),x_z_tank(end,:))
% plot(u_zmp(1,:),u_zmp(end,:))
plot(out.xout(:,4)'-moving_tank(1,:),out.xout(:,5)'-moving_tank(4,:))
% plot(ddxyz_com(1,:),ddxyz_com(2,:))
xlabel('x position (m)') 
ylabel('y position (m)') 
legend({'ZMP','COM'})
axis equal 
set(gca,'fontsize',18)%%

