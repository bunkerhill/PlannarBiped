close all

%% set up
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')
T=0:0.008:5;

%% data process
% cut double support
x_z = x_z_tank(:,26:end);
xout = out.xout(26:end,:);
moving = moving_tank(:,26:end);
stance = stance_leg(26:end);

% x_z = x_z_tank;
% xout = out.xout;
% moving = moving_tank;
% stance = stance_leg;

% sample
num = 2;
x_z = x_z(:,1:num:end);
xout = xout(1:num:end,:);
moving = moving(:,1:num:end);
stance = stance(:,1:num:end);
%% 3D plot
figure

for i=1:length(x_z(1,:))
    rectangle('Position',[x_z(1,i)-0.06,x_z(end,i)-0.01,0.12,0.02],'EdgeColor',"r")
    hold on
end

plot3(xout(:,4)'-moving(1,:),xout(:,5)'-moving(4,:),xout(:,6)','.',"Color","r")
for i=1:length(x_z(1,:))
    if(stance(i)==1)
        line([x_z(1,i) xout(i,4)'-moving(1,i)],[x_z(2,i) xout(i,5)'-moving(4,i)],[0 xout(i,6)'],"Color","b")
    else
        line([x_z(1,i) xout(i,4)'-moving(1,i)],[x_z(2,i) xout(i,5)'-moving(4,i)],[0 xout(i,6)'],"Color","y")
    end
end

[X,Y] = meshgrid(-0.5:0.1:3.5,-0.8:0.1:0.8);
Z = 0*X;
surf(X,Y,Z)

xlim([-0.5 3.5])
ylim([-0.8 0.8])
zlim([0 0.8])
axis equal

title('foot placement(ZMP)')
xlabel('x position (m)') 
ylabel('y position (m)') 
zlabel('z position (m)') 
legend({'actual zmp','actual com'})

%% 
moving_xy = [];
x = 0;
y = 0;
dx = 0;
dy = 0;
dt = 0.0005;
last_acc = [0;0];
for i=1:10000
    ddxy_s = random(:,i); % 0.56  0.58
    if abs(ddxy_s(1) - last_acc(1)) > 3
        ddxy_s(1) = last_acc(1) + sign((ddxy_s(1) - last_acc(1)))*3;
    end

    if abs(ddxy_s(2) - last_acc(2)) > 3
        ddxy_s(2) = last_acc(2) + sign((ddxy_s(2) - last_acc(2)))*3;
    end
    last_acc = ddxy_s;
    ddx = ddxy_s(1);
    ddy = ddxy_s(2);

    dx = dx + ddx*dt;
    x = x + dx*dt;
    dy = dy + ddy*dt;
    y = y + dy*dt;
    moving_xy = [moving_xy [x;dx;ddx;y;dy;ddy]];
end
figure
plot(moving_xy(1,:))
hold on
plot(moving_xy(4,:))

plot(moving_xy(2,:))
plot(moving_xy(5,:))

figure
plot(moving_xy(3,:))
hold on
plot(moving_xy(6,:))
legend({'x','y'})