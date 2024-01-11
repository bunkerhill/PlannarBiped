close all

figure
plot(x_z_tank(1,:))
hold on
plot(u_zmp_tank(1,:))

figure
plot(x_z_tank(end,:))
hold on
plot(u_zmp_tank(end,:))

figure
plot(x_z_tank(1,:),x_z_tank(end,:))
hold on
plot(u_zmp_tank(1,:),u_zmp_tank(end,:))
plot(out.xout(:,4),out.xout(:,5))
plot(ddxyz_com_tank(1,:),ddxyz_com_tank(2,:))