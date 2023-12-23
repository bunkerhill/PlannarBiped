%% Calculate the leg length and leg pitch angle
clear
clc
root = get_root_path();
addpath(root)

%% Two links system example

% Parameters
m1 = sym('m1'); % mass of first link
l1 = sym('l1'); % length of first link
com1 = Point2D('Y', -l1); % COM (center of mass) location [l1, 0]
I1 = sym('I1'); % moment of inertial around COM
m2 = sym('m2'); % mass of second link
l2 = sym('l2'); % length of second link
com2 = Point2D('Y', -l2); % COM (center of mass) location [l2, 0]
I2 = sym('I2'); % moment of inertial around COM

% General coordinates
theta1 = sym('theta1');
theta2 = sym('theta2');
generalCoordinates = [theta1, theta2];

% General Velocities
dtheta1 = sym('dtheta1');
dtheta2 = sym('dtheta2');
generalVelocities = [dtheta1, dtheta2];

%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

%% First link
link1 = CoordinateFrame2D('Name', 'link1', 'ParentFrame', base, 'Rotation', SO2(theta1));
link1Body = RigidBody2D('Name', 'link1', 'CoordinateFrame', link1, 'Mass', m1, 'CenterOfMassInBodyFrame', com1, 'MomentOfInertia', I1);

% link1COMPosition = link1Body.getCOMPosition()
% link1COMVelocity = link1Body.getCOMVelocity(generalCoordinates, generalVelocities)
% link1COMTransitionEnergy = simplify(link1Body.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities))
% link1RotationEnergy = link1Body.getRotationEnergy(generalCoordinates, generalVelocities)
% link1KineticEnergy = simplify(link1Body.getKineticEnergy(generalCoordinates, generalVelocities))
% link1PotentialEnergy = link1Body.getPotentialEnerge()

%% Second link
link2 = CoordinateFrame2D('Name', 'link2', 'ParentFrame', link1, 'Rotation', SO2(theta2), 'Displacement', Point2D('Y', -l1));
link2Body = RigidBody2D('Name', 'link2', 'CoordinateFrame', link2, 'Mass', m2, 'CenterOfMassInBodyFrame', com2, 'MomentOfInertia', I2);

link2COMPosition = simplify(link2Body.getCOMPosition())
% link2COMVelocity = simplify(link2Body.getCOMVelocity(generalCoordinates, generalVelocities))
% link2COMTransitionEnergy = link2Body.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities)
% link2RotationEnergy = link2Body.getRotationEnergy(generalCoordinates, generalVelocities)
% link2KineticEnergy = link2Body.getKineticEnergy(generalCoordinates, generalVelocities)
% link2PotentialEnergy=simplify(link2Body.getPotentialEnerge())

%% length
LL = sqrt(link2COMPosition(1)^2+link2COMPosition(2)^2);
simplify(LL)
%% pitch angle
LA = atan(link2COMPosition(1)/link2COMPosition(2));
simplify(LA)
