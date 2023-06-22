clear
clc
root = get_root_path();
addpath(root)

%% Seven links biped

% Parameters
m_torso = sym('m_torso'); % mass of torso
l_torso = sym('l_torso'); % length of torso
com_torso = Point2D('X', 0, 'Y', 0); % COM (center of mass) location [0, 0]
I_torso = sym('I_torso'); % moment of inertial around COM

m_thigh = sym('m_thigh'); % mass of thigh
l_thigh = sym('l_thigh'); % length of thigh
com_thigh = Point2D('Y', -l_thigh/2); % COM (center of mass) location [0, -l_thigh/2]
I_thigh = sym('I_thigh'); % moment of inertial around COM

m_shank = sym('m_shank'); % mass of shank
l_shank = sym('l_shank'); % length of shank
com_shank = Point2D('Y', -l_shank/2); % COM (center of mass) location [0, -l_shank/2]
I_shank = sym('I_shank'); % moment of inertial around COM

m_foot = sym('m_foot'); % mass of foot
l_foot = sym('l_foot'); % length of foot
com_foot = Point2D('Y', -l_foot/2); % COM (center of mass) location [0, -l_foot/2]
I_foot = sym('I_foot'); % moment of inertial around COM

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

% Torso 
torso = CoordinateFrame2D('Name', 'torso', 'ParentFrame', base, 'Rotation', SO2(theta0), 'Displacement', Point2D('X', body_x, 'Y', body_y));
torsoBody = RigidBody2D('Name', 'torso', 'CoordinateFrame', torso, 'Mass', m_torso, 'CenterOfMassInBodyFrame', com_torso, 'MomentOfInertia', I_torso);

% Left thigh
left_thigh = CoordinateFrame2D('Name', 'left_thigh', 'ParentFrame', torso, 'Rotation', SO2(theta1), 'Displacement', Point2D('Y', -l_torso/2));
left_thighBody = RigidBody2D('Name', 'left_thigh', 'CoordinateFrame', left_thigh, 'Mass', m_thigh, 'CenterOfMassInBodyFrame', com_thigh, 'MomentOfInertia', I_thigh);

% Left shank
left_shank = CoordinateFrame2D('Name', 'left_shank', 'ParentFrame', left_thigh, 'Rotation', SO2(theta2), 'Displacement', Point2D('Y', -l_thigh));
left_shankBody = RigidBody2D('Name', 'left_shank', 'CoordinateFrame', left_shank, 'Mass', m_shank, 'CenterOfMassInBodyFrame', com_shank, 'MomentOfInertia', I_shank);

% Left foot
left_foot = CoordinateFrame2D('Name', 'left_foot', 'ParentFrame', left_shank, 'Rotation', SO2(theta3), 'Displacement', Point2D('Y', -l_shank));
left_footBody = RigidBody2D('Name', 'left_foot', 'CoordinateFrame', left_foot, 'Mass', m_foot, 'CenterOfMassInBodyFrame', com_foot, 'MomentOfInertia', I_foot);

% Right thigh
right_thigh = CoordinateFrame2D('Name', 'right_thigh', 'ParentFrame', torso, 'Rotation', SO2(theta4), 'Displacement', Point2D('Y', -l_torso/2));
right_thighBody = RigidBody2D('Name', 'right_thigh', 'CoordinateFrame', right_thigh, 'Mass', m_thigh, 'CenterOfMassInBodyFrame', com_thigh, 'MomentOfInertia', I_thigh);

% Right shank
right_shank = CoordinateFrame2D('Name', 'right_shank', 'ParentFrame', right_thigh, 'Rotation', SO2(theta5), 'Displacement', Point2D('Y', -l_thigh));
right_shankBody = RigidBody2D('Name', 'right_shank', 'CoordinateFrame', right_shank, 'Mass', m_shank, 'CenterOfMassInBodyFrame', com_shank, 'MomentOfInertia', I_shank);

% Right foot
right_foot = CoordinateFrame2D('Name', 'right_foot', 'ParentFrame', right_shank, 'Rotation', SO2(theta6), 'Displacement', Point2D('Y', -l_shank));
right_footBody = RigidBody2D('Name', 'right_foot', 'CoordinateFrame', right_foot, 'Mass', m_foot, 'CenterOfMassInBodyFrame', com_foot, 'MomentOfInertia', I_foot);

%% Calculate Energy

% total kinetic energy
KE_torso = torsoBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_left_thigh = left_thighBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_left_shank = left_shankBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_left_foot = left_footBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_right_thigh = right_thighBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_right_shank = right_shankBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE_right_foot = right_footBody.getKineticEnergy(generalCoordinates, generalVelocities);
KE = KE_torso + KE_left_thigh + KE_left_shank + KE_left_foot + KE_right_thigh + KE_right_shank + KE_right_foot;

KE = simplify(KE)

% total potential energy

PE_torso = torsoBody.getPotentialEnerge();
PE_left_thigh = left_thighBody.getPotentialEnerge();
PE_left_shank = left_shankBody.getPotentialEnerge();
PE_left_foot = left_footBody.getPotentialEnerge();
PE_right_thigh = right_thighBody.getPotentialEnerge();
PE_right_shank = right_shankBody.getPotentialEnerge();
PE_right_foot = right_footBody.getPotentialEnerge();
PE = PE_torso + PE_left_thigh + PE_left_shank + PE_left_foot + PE_right_thigh + PE_right_shank + PE_right_foot;

PE = simplify(PE)

%% Calculate Dynamics Equation using Lagrange

% gravity vector
G_vect = jacobian(PE,generalCoordinates).';
G_vect = simplify(G_vect);

% mass-inertial matrix
D_mtx = simplify(jacobian(KE,generalVelocities).');
D_mtx = simplify(jacobian(D_mtx,generalVelocities));

% Coriolis and centrifugal matrix
syms C_mtx real
n=max(size(generalCoordinates));
for k=1:n
	for j=1:n
		C_mtx(k,j)=0;
		for i=1:n
			C_mtx(k,j)=C_mtx(k,j)+1/2*(diff(D_mtx(k,j),generalCoordinates(i)) + ...
				diff(D_mtx(k,i),generalCoordinates(j)) - ...
				diff(D_mtx(i,j),generalCoordinates(k)))*generalVelocities(i);
		end
	end
end
C_mtx=simplify(C_mtx);

% input matrix
Phi_0 = [theta1;
         theta2;
         theta3;
         theta4;
         theta5;
         theta6];
B_mtx = jacobian(Phi_0,generalCoordinates).';

% swing foot force input matrix (F_ext = [F_x;F_y;M_z])
% ºŸ…Ë÷ß≥≈Õ»Œ™”“Õ»
Phi_1 = [right_footBody.getCOMPosition(); right_footBody.getCOMAngle()];
E_mtx = jacobian(Phi_1,generalCoordinates).';