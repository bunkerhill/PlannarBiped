function xi_out = LIPModel(xi_in, omega, dt, stanceFootPosition, disturbance)
%LIPMODEL Summary of this function goes here
%   Detailed explanation goes here
xi_out = zeros(2,1);
xe = zeros(2,1);
omegaSquared = omega*omega;
xe(1) = stanceFootPosition(1)+1/omegaSquared*disturbance(1);
xe(2) = stanceFootPosition(2)+1/omegaSquared*disturbance(2);
xi_out(1)=(xi_in(1)-xe(1))*exp(omega*dt)+xe(1);
xi_out(2)=(xi_in(2)-xe(2))*exp(omega*dt)+xe(2);
end