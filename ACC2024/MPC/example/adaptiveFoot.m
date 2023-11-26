close all
clear
clc
g=9.8;
h=0.231;
omega = sqrt(g/h);

%% LIP dynamics
A = [0, 1;
     omega^2, 0];
B = [0; -omega^2];

%% DCM dynamics
xi_dyn = @(t, xi0, z)(exp(omega*t)*(xi0 - z)+z);
%% Parameters
% Initial value of DCM
xi0 = 0.06;
% Initial ZMP
z0 = 0;
% Step Duration in seconds
stepDuration = 0.2;
% Step length in meters
stepLength = (xi0-z0)*(exp(omega*stepDuration)-1)

stepLengthMax = 0.18;

t = 0:0.01:stepDuration;

xiTrajectory = [];
zmpTrajectory = [];
timeTrajectory = [];

for i=1:10
    if i == 1
        xi = xi_dyn(t, xi0, z0);
        zmpTrajectory = [zmpTrajectory z0*ones(size(t))];
        xiTrajectory = [xiTrajectory xi];
        timeTrajectory = [timeTrajectory t];
        t = t+stepDuration;
    else
        z_n = zmpTrajectory(end) + min(stepLength, stepLengthMax);
        xi = xi_dyn(t-t(1), xiTrajectory(end), z_n);
        zmpTrajectory = [zmpTrajectory z_n*ones(size(t))];
        xiTrajectory = [xiTrajectory xi];
        timeTrajectory = [timeTrajectory t];
        t = t+stepDuration;
    end
end


figure, plot(timeTrajectory, xiTrajectory,'o')
hold on, plot(timeTrajectory,zmpTrajectory,'.')

figure, plot(timeTrajectory, xiTrajectory-zmpTrajectory)


