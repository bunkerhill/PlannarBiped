function [L,T,V]= getLagrangian(bodyObjName,generalCoordinates,generalVelocities)
% test the additional function that obtains the Lagrangian,L= T(kinetic
% energy) - V(potential energy)
% input: rigid body object;generalCoordinates;generalVelocities
% output: L: Larangian; T: kinetic energy; V: potential energy

% get COM position & velocity 
bodyCOM_Postion = bodyObjName.getCOMPosition();
bodyCOM_Vel = bodyObjName.getCOMVelocity(generalCoordinates, generalVelocities);

% get kinetic energy
translationKineEnergy = bodyObjName.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities);
rotationKineEnergy = bodyObjName.getRotationEnergy(generalCoordinates, generalVelocities);

T = simplify(translationKineEnergy + rotationKineEnergy);

% get potential energy
V = simplify(bodyObjName.getPotentialEnerge());

% get Larangian, L = T - V
L = T - V;
end