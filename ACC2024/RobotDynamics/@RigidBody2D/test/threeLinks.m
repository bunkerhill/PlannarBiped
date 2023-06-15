clear
clc
root = get_root_path();
addpath(root)

%% Three links system example

% Parameters
m1 = sym('m1'); % mass of first link
l1 = sym('l1'); % length of first link
com1 = Point2D('Y', -l1/2); % COM (center of mass) location [0, -l1/2]
I1 = sym('I1'); % moment of inertial around COM
m2 = sym('m2'); % mass of second link
l2 = sym('l2'); % length of second link
com2 = Point2D('Y', -l2/2); % COM (center of mass) location [0, -l2/2]
I2 = sym('I2'); % moment of inertial around COM
m3 = sym('m3'); % mass of third link
l3 = sym('l3'); % length of third link
com3 = Point2D('Y', -l3/2); % COM (center of mass) location [0, -l3/2]
I3 = sym('I3'); % moment of inertial around COM

% General coordinates
theta1 = sym('theta1');
theta2 = sym('theta2');
theta3 = sym('theta3');
generalCoordinates = [theta1, theta2, theta3];

% General Velocities
dtheta1 = sym('dtheta1');
dtheta2 = sym('dtheta2');
dtheta3 = sym('dtheta3');
generalVelocities = [dtheta1, dtheta2, dtheta3];

%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

%% First link
link1 = CoordinateFrame2D('Name', 'link1', 'ParentFrame', base, 'Rotation', SO2(theta1));
link1Body = RigidBody2D('Name', 'link1', 'CoordinateFrame', link1, 'Mass', m1, 'CenterOfMassInBodyFrame', com1, 'MomentOfInertia', I1);

link1COMPosition = link1Body.getCOMPosition()
link1COMVelocity = link1Body.getCOMVelocity(generalCoordinates, generalVelocities)
link1COMTransitionEnergy = simplify(link1Body.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities))
link1RotationEnergy = link1Body.getRotationEnergy(generalCoordinates, generalVelocities)
link1KineticEnergy = simplify(link1Body.getKineticEnergy(generalCoordinates, generalVelocities))
link1PotentialEnergy = link1Body.getPotentialEnerge()

%% Second link
link2 = CoordinateFrame2D('Name', 'link2', 'ParentFrame', link1, 'Rotation', SO2(theta2), 'Displacement', Point2D('Y', -l1))
link2Body = RigidBody2D('Name', 'link2', 'CoordinateFrame', link2, 'Mass', m2, 'CenterOfMassInBodyFrame', com2, 'MomentOfInertia', I2);

link2COMPosition = simplify(link2Body.getCOMPosition())
link2COMVelocity = simplify(link2Body.getCOMVelocity(generalCoordinates, generalVelocities))
link2COMTransitionEnergy = link2Body.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities)
link2RotationEnergy = link2Body.getRotationEnergy(generalCoordinates, generalVelocities)
link2KineticEnergy = link2Body.getKineticEnergy(generalCoordinates, generalVelocities)
link2PotentialEnergy=simplify(link2Body.getPotentialEnerge())

%% Third link
link3 = CoordinateFrame2D('Name', 'link3', 'ParentFrame', link2, 'Rotation', SO2(theta3), 'Displacement', Point2D('Y', -l2))
link3Body = RigidBody2D('Name', 'link3', 'CoordinateFrame', link3, 'Mass', m3, 'CenterOfMassInBodyFrame', com3, 'MomentOfInertia', I3);

link3COMPosition = simplify(link3Body.getCOMPosition())
link3COMVelocity = simplify(link3Body.getCOMVelocity(generalCoordinates, generalVelocities))
link3COMTransitionEnergy = link3Body.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities)
link3RotationEnergy = link3Body.getRotationEnergy(generalCoordinates, generalVelocities)
link3KineticEnergy = link3Body.getKineticEnergy(generalCoordinates, generalVelocities)
link3PotentialEnergy=simplify(link3Body.getPotentialEnerge())
