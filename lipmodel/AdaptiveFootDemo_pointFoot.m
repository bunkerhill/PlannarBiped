clear
clc
close all
addpath(genpath("./casadi-3"))


comHeight=0.525;% center of mass height
stepDuration=0.2;%sec
averageSpeed=0.4;%m/s
stepWidth=0.2;%m
g=9.8;%m/s^2
omega=sqrt(g/comHeight);

footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
% obj.drawPeriodicGait(5);
Nsteps=3; % number of steps that planner plans ahead
currentStanceFootID=0; % 0 means left foot is stance foot. 1 means right foot is stance foot 
currentStanceFootPosition=[0.0; 0.1];
xi=[0.03;0];% divergent component of motion x,y
currentZMP = [0;0.1];
timeVector=[];
xiVector=[];
zmpVector=[];
totalTime = 400;
stepDurationTic=round(stepDuration*100);

for i=1:totalTime
    currentTime=0.01*i;
    timeVector=[timeVector, currentTime];
    footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime);
    xiVector=[xiVector, xi];
    % if mod(i,40) == 0
    %     footPlanner.drawOptimalFootPlacement()
    % end
    % footPlanner.drawPeriodicGait(7)
    xi(1)=(xi(1)-currentStanceFootPosition(1))*exp(omega*0.01)+currentStanceFootPosition(1);
    xi(2)=(xi(2)-currentStanceFootPosition(2))*exp(omega*0.01)+currentStanceFootPosition(2);
    zmpVector = [zmpVector, currentStanceFootPosition];

    currentStanceFootID = mod(floor((i+1)/stepDurationTic),2);
    prevStanceFootID = mod(floor((i)/stepDurationTic),2);
    if currentStanceFootID == prevStanceFootID
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(1); footPlanner.stanceFootConstraint.ankleY(1)];
    else
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(2); footPlanner.stanceFootConstraint.ankleY(2)];
    end
    
    currentStanceFootPosition = nextStanceFootPosition;
end

figure,plot(timeVector,zmpVector(1,:),'o-')
hold on,plot(timeVector, xiVector(1,:),'.-')
legend("zmp x","xiux");
figure,plot(timeVector,zmpVector(2,:),'o-')
hold on,plot(timeVector, xiVector(2,:),'.-')
legend("zmp y","xiuy");
