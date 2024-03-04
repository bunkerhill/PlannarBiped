clear
clc
close all


comHeight=0.525;% center of mass height
stepDuration=0.2;%sec
averageSpeed=0.4;%m/s
stepWidth=0.2;%m
g=9.8;%m/s^2
omega=sqrt(g/comHeight);

footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
% obj.drawPeriodicGait(5);
Nsteps=5; % number of steps that planner plans ahead
currentStanceFootID=0; % 0 means left foot is stance foot. 1 means right foot is stance foot 
currentStanceFootPosition=[0.0; 0.1];
xi=[0.03;0];% divergent component of motion x,y
currentZMP = [0;0.1];
timeVector=[];
xiVector=[];
zmpVector=[];
for i=1:1000
    if mod(i,20) == 0
        % assume the swing leg end at desired second foot placement from footPlanner
        currentStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(2);footPlanner.stanceFootConstraint.ankleY(2)];
        currentStanceFootID = mod(floor(i/20),2);
    end
    currentTime=0.01*i;
    timeVector=[timeVector, currentTime];
    footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime);
    xiVector=[xiVector, xi];
    % footPlanner.drawOptimalFootPlacement()
    footHalfLength=0.06;
    footHalfWidth=0.01;
    zmpController = intrinsicMPC(comHeight, footHalfLength, footHalfWidth);
    zmpVector=[zmpVector, currentZMP];
    zmpController = zmpController.MPC(xi, currentZMP, currentTime, footPlanner.stanceFootConstraint);
    % zmpController.drawZMPPreviewAndConstraint()
    % footPlanner.drawPeriodicGait(5)
    optimalZMP = zmpController.getOptimalZMP();
    xi(1)=(xi(1)-optimalZMP(1))*exp(omega*0.01)+optimalZMP(1);
    xi(2)=(xi(2)-optimalZMP(2))*exp(omega*0.01)+optimalZMP(2);
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
axis equal
