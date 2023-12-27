close all
x_com = out.xout(:,4);
y_com = out.xout(:,5);
z_com = out.xout(:,6);

rfoot = out.fpfv(:,1:3);
lfoot = out.fpfv(:, 7:9);

figure, plot(out.time, lfoot(:,3),"b")
hold on,plot(out.time, rfoot(:,3), "r")
legend('lfoot z', 'rfoot z')

% figure, plot(out.time, x_com,"k")
% hold on, plot(out.time, lfoot(:,1),"b")
% hold on, plot(out.time, rfoot(:,1),"r")
% legend('x com', 'l foot', 'r foot')
% xlabel("time (s)")
% ylabel("x direction")
% 
% figure, plot(out.time, y_com, "k")
% hold on, plot(out.time, lfoot(:,2), "b")
% hold on, plot(out.time, rfoot(:,2), 'r')
% xlabel("time (s)")
% ylabel("y direction")
% legend('y com', 'l foot', 'r foot')

figure, plot(x_com, y_com, '.')
xlabel("x com")
ylabel("y com")
legend('com position')
xlabel("x (m)")
ylabel("y (m)")
% axis equal

% ground reaction force
timeGRF = 0.001*(0:1:size(out.MPCForce,1)-1);
rGRF = out.MPCForce(:, 1:3);
lGRF = out.MPCForce(:, 4:6);
% ground reaction moment
rGRM = out.MPCForce(:, 7:9);
lGRM = out.MPCForce(:, 10:12);

% figure, plot(timeGRF, lGRF(:,3),"b")
% hold on,plot(timeGRF, rGRF(:,3),"r")
% legend("lGRFz", "rGRFz")
% 
% figure, plot(timeGRF, lGRF(:,1),"b")
% hold on,plot(timeGRF, rGRF(:,1),"r")
% legend("lGRFx", "rGRFx")
% 
% figure, plot(timeGRF, lGRF(:,2),"b")
% hold on,plot(timeGRF, rGRF(:,2),"r")
% legend("lGRFy", "rGRFy")
% 
% figure, plot(timeGRF, lGRM(:, 1), "b")
% hold on,plot(timeGRF, rGRM(:, 1), "r")
% 
% figure, plot(timeGRF, lGRM(:, 2), "b")
% hold on,plot(timeGRF, rGRM(:, 2), "r")
% 
% figure, plot(timeGRF, lGRM(:, 3), "b")
% hold on,plot(timeGRF, rGRM(:, 3), "r")


stanceFoot = zeros(1, length(timeGRF));
stanceFootPosition = zeros(length(timeGRF), 3);
zmp_x = zeros(1, length(timeGRF));
zmp_y = zeros(1, length(timeGRF));

for i = 1:length(timeGRF)
    if lGRF(i,3) > rGRF(i,3)
        % left foot is stance foot
        stanceFoot(i) = 1;
        stanceFootPosition(i,:) = interp1(out.time, lfoot,timeGRF(i));
        zmp_x(i) = stanceFootPosition(i,1) - lGRM(i,2)/lGRF(i,3);
        zmp_y(i) = stanceFootPosition(i,2) - lGRM(i,1)/lGRF(i,3);
    else
        % right foot is stance foot
        stanceFoot(i) = 0;
        stanceFootPosition(i,:) = interp1(out.time, rfoot,timeGRF(i));
        zmp_x(i) = stanceFootPosition(i,1) - rGRM(i,2)/rGRF(i,3);
        zmp_y(i) = stanceFootPosition(i,2) - rGRM(i,1)/rGRF(i,3);
    end
end
figure,
hold on, plot(timeGRF, stanceFootPosition(:,1),'r.')
% hold on, plot(timeGRF, stanceFootPosition(:,2), 'k.')
hold on, plot(timeGRF, zmp_x, 'm')
hold on, plot(out.time, x_com, 'b')
legend('stance foot position', 'zmp x', 'com x')
xlabel('time (s)')
ylabel('x direction')

figure, plot(timeGRF, stanceFootPosition(:,2), 'r.')
hold on,plot(timeGRF, zmp_y, 'm')
hold on, plot(out.time, y_com, 'b')
legend('stance foot position', 'zmp y', 'com y')
xlabel('time (s)')
ylabel('y direction')

figure, plot(out.time, z_com)
legend('com height')
xlabel('time (s)')
ylabel('z direction')
% figure,plot(stanceFootPosition(:,1), stanceFootPosition(:,2), 'r.')
% hold on,plot(x_com, y_com, 'b')
% hold on,plot(zmp_x, zmp_y,'k')
% hold on,plot(rfoot(:,1), rfoot(:,2))
% hold on,plot(lfoot(:,1), lfoot(:,2))

figure, plot(out.time, out.qout(:,2))
hold on,plot(out.time, out.qout(:,7))
legend('right hip yaw', 'left hip yaw')
