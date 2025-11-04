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

% footPlanner=adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
footPlanner=scenairoTree_adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
% footPlanner=KnownFuture_adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);
% obj.drawPeriodicGait(5);
Nsteps=3; % number of steps that planner plans ahead
currentStanceFootID=0; % 0 means left foot is stance foot. 1 means right foot is stance foot 
currentStanceFootPosition=[0.0; 0.1];
xi=[0.03;0];% divergent component of motion x,y
currentZMP = [0;0.1];
timeVector=[];
xiVector=[];
zmpVector=[];
disturbanceVector=[];
surfaceMotionVector=[];
totalTime = 200;
deltaT=0.01;
stepDurationTic=round(stepDuration*100);

distbancePeriod = 2;
Amplitude=0.19;
sinFunc = @(t) Amplitude*sin(2*pi/distbancePeriod*t);

for i=1:totalTime
    currentTime=deltaT*i;
    timeVector=[timeVector, currentTime];
    xiVector=[xiVector, xi];
    % if mod(i,40) == 0
    %     footPlanner.drawOptimalFootPlacement()
    % end
    % footPlanner.drawPeriodicGait(7)

    surfaceMotion_x = sinFunc(currentTime);
    surfaceMotionVector = [surfaceMotionVector [surfaceMotion_x;0]];

    disturbance_x = -(2*pi/distbancePeriod)^2*sinFunc(currentTime);
    disturbance = [disturbance_x;0];
    % footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime);
    footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime,disturbance);
    % footPlanner=footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime,-(2*pi/distbancePeriod)^2*Amplitude,2*pi/distbancePeriod);
    xi = LIPModel(xi, omega, deltaT, currentStanceFootPosition, disturbance);
    disturbanceVector = [disturbanceVector disturbance];
    zmpVector = [zmpVector, currentStanceFootPosition];

    currentStanceFootID = mod(floor((i+1)/stepDurationTic),2);
    prevStanceFootID = mod(floor((i)/stepDurationTic),2);
    if currentStanceFootID == prevStanceFootID
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(1); footPlanner.stanceFootConstraint.ankleY(1)];
    else
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(2); footPlanner.stanceFootConstraint.ankleY(2)];
    end
    
    currentStanceFootPosition = nextStanceFootPosition;

    if abs(xi(1)-currentStanceFootPosition(1)) > 0.3 
        % if dcm offset is higher than this value, consider the robot falls
        fprintf("currentTime: %f, %s \n", currentTime, "fall due to x")
        break;
    end

    if abs(xi(2)-currentStanceFootPosition(2)) > 0.3
        % if dcm offset is higher than this value, consider the robot falls
        fprintf("currentTime: %f, %s \n", currentTime, "fall due to y")
        break;
    end

end

figure,plot(timeVector,zmpVector(1,:),'o-')
hold on,plot(timeVector, xiVector(1,:),'.-')
hold on,plot(timeVector, xiVector(1,:)-zmpVector(1,:),'.-')
legend("zmp x","xiux", "x dcm offset");
xlabel("t(sec)")
ylabel("x(m)")

figure,plot(timeVector,zmpVector(2,:),'o-')
hold on,plot(timeVector, xiVector(2,:),'.-')
hold on,plot(timeVector, xiVector(2,:)-zmpVector(2,:),'.-')
legend("zmp y","xiuy", "y dcm offset");
xlabel("t(sec)")
ylabel("y(m)")


figure,plot(timeVector, disturbanceVector(1,:),'.-')
hold on,plot(timeVector, disturbanceVector(2,:),'.-')
hold on,plot(timeVector, surfaceMotionVector(1,:),'.-')
xlabel("t(sec)")
ylabel("x/y acceleration (m/s/s)")
legend("x acceleration", "y acceleration", "x position")

figure,plot(zmpVector(1,:),zmpVector(2,:),'o-')
% legend("zmp x","xiux", "x dcm offset");
xlabel("x(m)")
ylabel("y(m)")
