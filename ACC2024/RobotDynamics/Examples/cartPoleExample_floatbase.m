clear
clc

%% Cart-Pole system example

% Parameters
m1 = sym('m1'); % mass of the cart
m2 = sym('m2'); % mass of the pole
l2 = sym('l2'); % length of the pole
% com2 = Point2D('Y', l2/2); % COM (center of mass) location on pole [0, l2/2]
I2 = sym('I2'); % pole's moment of inertia around com2

% General Coordinates
x = sym('x'); % pendulum COM position x-axis
y = sym('y'); % pendulum COM position y-axis
theta = sym('theta'); % pendulum's roll angle around z-axis
generalCoordinates = [x, y, theta];

% General Velocities
vx = sym('vx'); % pendulum COM velocity along x-axis
vy = sym('vy'); % pendulum COM velocity along y-axis
dtheta = sym('dtheta'); % pole's roll angle velocity
generalVelocities = [vx, vy, dtheta];


%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

%% Pole Frame: [x,y,theta] relative to base
p = Point2D('X', x, 'Y', y);
poleFrame = CoordinateFrame2D('Name', 'pole', 'ParentFrame', base, 'Displacement', p, 'Rotation', SO2(theta));
poleBody = RigidBody2D('Name', 'pole', 'CoordinateFrame', poleFrame, 'Mass', m2, 'CenterOfMassInBodyFrame', Point2D(), 'MomentOfInertia', I2);


%% Cart Frame:
cartFrame = CoordinateFrame2D('Name', 'cart', 'ParentFrame', poleFrame, 'Displacement', Point2D('Y', -l2/2));
cartBody = RigidBody2D('Name', 'cart', 'CoordinateFrame', cartFrame, 'Mass', m1, 'CenterOfMassInBodyFrame', Point2D('X', 0, 'Y', 0), 'MomentOfInertia', 0);


%% Actuation: Horizontal force is applied on cart
InputForce = Force2D('X', sym('F_x'));
ForceAppliedPoint = Point2D('X', x+l2/2*sin(theta), 'Y', y-l2/2*cos(theta));
InputGeneralizedForce = GeneralizedForce2D('Name', 'Input', 'Force', InputForce, 'Displacement', ForceAppliedPoint);

%% Contact Constraints: Virtical force is constraint force
ContactConstraint = Contact2D('Name', 'yDirection', 'CoordinateFrame', cartFrame, 'ConstraintType', ["y"]);

contactPoint = ContactConstraint.getContactPoint()
% contactJacobian = ContactConstraint.getContactJacobian()


%% Build continous dynamics
cartPoleSystem = ContinuousDynamics(generalCoordinates, generalVelocities);
cartPoleSystem = cartPoleSystem.AddRigidBody(poleBody);
cartPoleSystem = cartPoleSystem.AddRigidBody(cartBody);

cartPoleSystem = cartPoleSystem.AddGeneralizedForce(InputGeneralizedForce);

cartPoleSystem = cartPoleSystem.AddContactConstraints(ContactConstraint);

cartPoleSystem.getTotalKineticEnergy()
cartPoleSystem.getTotalPotentialEnergy()


Mass = cartPoleSystem.getMassMatrix()
Gravity = cartPoleSystem.getGravityVector()
Coriolis = cartPoleSystem.getCoriolisVector()
Force = cartPoleSystem.getGeneralizedForceVector()
BMatrix = cartPoleSystem.getInputMatrix(InputForce.X)
J = cartPoleSystem.getContactJacobian()
Jdot = cartPoleSystem.getJdot()
Fy = cartPoleSystem.getConstraintForce()