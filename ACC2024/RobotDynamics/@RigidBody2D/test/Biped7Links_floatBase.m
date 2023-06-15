clear
clc
root = get_root_path();
addpath(root)

%% Seven links biped

% Parameters
m0 = sym('m0'); % mass of body link
l0 = sym('l0'); % length of body link
com0 = Point2D('X', 0, 'Y', 0); % COM (center of mass) location [0, 0]
I0 = sym('I0'); % moment of inertial around COM

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

m4 = sym('m4');
l4 = sym('l4'); 
com4 = Point2D('Y', -l4/2); % COM (center of mass) location [0, -l4/2]
I4 = sym('I4'); % moment of inertial around COM
m5 = sym('m5');  
l5 = sym('l5'); 
com5 = Point2D('Y', -l5/2); % COM (center of mass) location [0, -l5/2]
I5 = sym('I5'); % moment of inertial around COM
m6 = sym('m6'); 
l6 = sym('l6');
com6 = Point2D('Y', -l6/2); % COM (center of mass) location [0, -l6/2]
I6 = sym('I6'); % moment of inertial around COM

% General coordinates
body_x = sym('body_x');
body_y = sym('body_y');
theta0 = sym('theta0');
theta1 = sym('theta1');
theta2 = sym('theta2');
theta3 = sym('theta3');
theta4 = sym('theta4');
theta5 = sym('theta5');
theta6 = sym('theta6');
generalCoordinates = [body_x, body_y, theta0, theta1, theta2, theta3, theta4, theta5, theta6];

% General Velocities
dbody_x = sym('dbody_x');
dbody_y = sym('dbody_y');
dtheta0 = sym('dtheta0');
dtheta1 = sym('dtheta1');
dtheta2 = sym('dtheta2');
dtheta3 = sym('dtheta3');
dtheta4 = sym('dtheta4');
dtheta5 = sym('dtheta5');
dtheta6 = sym('dtheta6');
generalVelocities = [dbody_x, dbody_y, dtheta0, dtheta1, dtheta2, dtheta3, dtheta4, dtheta5, dtheta6];

%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

%% Seven Links (float base)

% Body 
link0 = CoordinateFrame2D('Name', 'link0', 'ParentFrame', base, 'Rotation', SO2(theta0), 'Displacement', Point2D('X', body_x, 'Y', body_y));
link0Body = RigidBody2D('Name', 'link0', 'CoordinateFrame', link0, 'Mass', m0, 'CenterOfMassInBodyFrame', com0, 'MomentOfInertia', I0);

% Left thigh
link1 = CoordinateFrame2D('Name', 'link1', 'ParentFrame', link0, 'Rotation', SO2(theta1), 'Displacement', Point2D('Y', -l0/2));
link1Body = RigidBody2D('Name', 'link1', 'CoordinateFrame', link1, 'Mass', m1, 'CenterOfMassInBodyFrame', com1, 'MomentOfInertia', I1);

% Left shank
link2 = CoordinateFrame2D('Name', 'link2', 'ParentFrame', link1, 'Rotation', SO2(theta2), 'Displacement', Point2D('Y', -l1));
link2Body = RigidBody2D('Name', 'link2', 'CoordinateFrame', link2, 'Mass', m2, 'CenterOfMassInBodyFrame', com2, 'MomentOfInertia', I2);

% Left foot
link3 = CoordinateFrame2D('Name', 'link3', 'ParentFrame', link2, 'Rotation', SO2(theta3), 'Displacement', Point2D('Y', -l2));
link3Body = RigidBody2D('Name', 'link3', 'CoordinateFrame', link3, 'Mass', m3, 'CenterOfMassInBodyFrame', com3, 'MomentOfInertia', I3);

% Right thigh
link4 = CoordinateFrame2D('Name', 'link4', 'ParentFrame', link0, 'Rotation', SO2(theta4), 'Displacement', Point2D('Y', -l0/2));
link4Body = RigidBody2D('Name', 'link4', 'CoordinateFrame', link4, 'Mass', m4, 'CenterOfMassInBodyFrame', com4, 'MomentOfInertia', I4);

% Right shank
link5 = CoordinateFrame2D('Name', 'link5', 'ParentFrame', link4, 'Rotation', SO2(theta5), 'Displacement', Point2D('Y', -l4));
link5Body = RigidBody2D('Name', 'link5', 'CoordinateFrame', link5, 'Mass', m5, 'CenterOfMassInBodyFrame', com5, 'MomentOfInertia', I5);

% Right foot
link6 = CoordinateFrame2D('Name', 'link6', 'ParentFrame', link5, 'Rotation', SO2(theta6), 'Displacement', Point2D('Y', -l5));
link6Body = RigidBody2D('Name', 'link6', 'CoordinateFrame', link6, 'Mass', m6, 'CenterOfMassInBodyFrame', com6, 'MomentOfInertia', I6);
