clear
clc
close all
set(groot, 'defaulttextinterpreter','latex')
set(groot, 'defaultaxesticklabelinterpreter','latex')
set(groot, 'defaultlegendinterpreter','latex')

ddxy_s_max = [15;15];
ddxy_s_min = [-15;-15];

comHeight=0.525;% center of mass height
stepDuration=0.2;%sec
averageSpeed=0.5;%m/s
stepWidth=0.2;%m
g=9.8;%m/s^2
omega=sqrt(g/comHeight);

footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth, ddxy_s_max, ddxy_s_min);
% obj.drawPeriodicGait(5);
Nsteps=5; % number of steps that planner plans ahead
currentStanceFootID=0; % 0 means left foot is stance foot. 1 means right foot is stance foot 
currentStanceFootPosition=[0.0; 0.1];
xi=[0;0];% divergent component of motion x,y
x_com = [0;0;0;0];
currentZMP = [0;0.1];
timeVector=[];
xiVector=[];
zmpVector=[];
comVector=[];
for i=1:300
    if mod(i,20) == 0
        % assume the swing leg end at desired second foot placement from footPlanner
        currentStanceFootPosition = [footPlanner.stanceFootConstraint.Up_ankleX(2);footPlanner.stanceFootConstraint.Up_ankleY(2)];
        currentStanceFootID = mod(floor(i/20),2);
        footPlanner.drawOptimalFootPlacement()
        zmpController.drawZMPPreviewAndConstraint()
    end
    currentTime=0.01*i;
    timeVector=[timeVector, currentTime];
    footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime,[1;0]);
    xiVector=[xiVector, xi];
    comVector=[comVector x_com];
    % footPlanner.drawOptimalFootPlacement()
    footHalfLength=0.06;
    footHalfWidth=0.01;
    zmpController = contingencyMPC(comHeight, footHalfLength, footHalfWidth, ddxy_s_max, ddxy_s_min);
    zmpVector=[zmpVector, currentZMP];
    zmpController = zmpController.MPC(xi, currentZMP, currentTime, footPlanner.stanceFootConstraint,[1;0]);
    % zmpController.drawZMPPreviewAndConstraint()
    % footPlanner.drawPeriodicGait(5)
    optimalZMP = zmpController.getOptimalZMP();
    x_com = lip_dynamics(x_com,currentStanceFootPosition,[1;0],0.01,comHeight,g);
    xi(1)=x_com(1)+x_com(2)/omega;
    xi(2)=x_com(3)+x_com(4)/omega;
    % xi(1)=(xi(1)-optimalZMP(1))*exp(omega*0.01)+optimalZMP(1);
    % xi(2)=(xi(2)-optimalZMP(2))*exp(omega*0.01)+optimalZMP(2);
    % currentZMP=currentStanceFootPosition;
    currentZMP=optimalZMP;
end
% footPlanner.drawOptimalFootPlacement()
% zmpController.drawZMPPreviewAndConstraint()
% figure,plot(timeVector,zmpVector(1,:),'-')
% hold on,plot(timeVector, xiVector(1,:),'-')
% legend("zmp x","\xi_u^x");
% figure,plot(timeVector,zmpVector(2,:),'-')
% hold on,plot(timeVector, xiVector(2,:),'-')
% legend("zmp y","\xi_u^y");

figure,plot(zmpVector(1,:),zmpVector(2,:),'-')
hold on,plot(comVector(1,:),comVector(3,:),'-')
plot(xiVector(1,:),xiVector(2,:))
legend("zmp","com","\xi_u");
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
