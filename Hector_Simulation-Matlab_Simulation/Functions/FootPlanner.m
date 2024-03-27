function [t] = FootPlanner(t)
% global variables
global footPlanner next_footHold i_gait Leftfoot Rightfoot com_x com_dx moving_xy

ddxy_s = [moving_xy(3);moving_xy(6)];
currentTime = t;
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

footPlanner = footPlanner.findOptimalFootPlacement(Nsteps,xi,currentStanceFootID,currentStanceFootPosition,currentTime,ddxy_s);
next_footHold = [footPlanner.stanceFootConstraint.Up_ankleX(2);footPlanner.stanceFootConstraint.Up_ankleY(2)];

end