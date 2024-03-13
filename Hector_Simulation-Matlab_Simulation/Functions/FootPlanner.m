function [t] = FootPlanner(t)
% global variables
global footPlanner next_footHold i_gait Leftfoot Rightfoot com_x com_dx

% stand on two feet first 0.2s
if t < 0.2
    % do nothing
    next_footHold = [0;0];
% walk
else

    currentTime = t - 0.2;
    Nsteps = 5; % number of steps that planner plans ahead
    comHeight=0.525;
    g=9.8;%m/s^2
    omega=sqrt(g/comHeight);

    if (i_gait==0) % R stance
        currentStanceFootID = 1; % 1 means right foot is stance foot 
        foot = Rightfoot;
    else
        currentStanceFootID = 0; % 0 means left foot is stance foot. 
        foot = Leftfoot;
    end
    
    currentStanceFootPosition = foot;
    xi = com_x + com_dx/omega;% divergent component of motion x,y
    
    footPlanner = footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime);
    next_footHold = [footPlanner.stanceFootConstraint.ankleX(2);footPlanner.stanceFootConstraint.ankleY(2)];
end