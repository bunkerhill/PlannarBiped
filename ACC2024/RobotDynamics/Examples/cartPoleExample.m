clear
clc

%% Cart-Pole system example

% Parameters
m1 = sym('m1'); % mass of the cart
m2 = sym('m2'); % mass of the pole
l2 = sym('l2'); % length of the pole
com2 = Point2D('Y', l2/2); % COM (center of mass) location on pole [0, l2/2]
I2 = sym('I2'); % pole's moment of inertia around com2

% General Coordinates
x = sym('x'); % cart position along x-axis
theta = sym('theta'); % pole's roll angle around z-axis
generalCoordinates = [x, theta];

% General Velocities
v = sym('v'); % cart velocity along x-axis
dtheta = sym('dtheta'); % pole's roll angle velocity
generalVelocities = [v, dtheta];


%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

%% Cart Frame: x axis transition 
p = Point2D('X', x);
cartFrame = CoordinateFrame2D('Name', 'cart', 'ParentFrame', base, 'Displacement', p);
cartBody = RigidBody2D('Name', 'cart', 'CoordinateFrame', cartFrame, 'Mass', m1, 'CenterOfMassInBodyFrame', Point2D('X', 0, 'Y', 0), 'MomentOfInertia', 0);

%% Pole Frame: z axis rotation
pole = CoordinateFrame2D('Name', 'pole', 'ParentFrame', cartFrame, 'Rotation', SO2(theta));
poleBody = RigidBody2D('Name', 'pole', 'CoordinateFrame', pole, 'Mass', m2, 'CenterOfMassInBodyFrame', com2, 'MomentOfInertia', I2);

%% Actuation: Horizontal force applied on cart
InputForce = Force2D('X', sym('F'));
InputGeneralizedForce = GeneralizedForce2D('Name', 'Input', 'Force', InputForce, 'Displacement', p);

%% Build continous dynamics
cartPoleSystem = ContinuousDynamics(generalCoordinates, generalVelocities);
cartPoleSystem = cartPoleSystem.AddRigidBody(cartBody);
cartPoleSystem = cartPoleSystem.AddRigidBody(poleBody);

cartPoleSystem = cartPoleSystem.AddGeneralizedForce(InputGeneralizedForce);


cartPoleSystem.getTotalKineticEnergy()


Mass = cartPoleSystem.getMassMatrix()
Gravity = cartPoleSystem.getGravityVector()
Coriolis = cartPoleSystem.getCoriolisVector()
Force = cartPoleSystem.getGeneralizedForceVector()
