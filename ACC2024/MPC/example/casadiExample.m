clear
clc

addpath('../../../casadi-3/')

%% Example of using Casadi
% https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi
% https://www.youtube.com/playlist?list=PLK8squHT_Uzej3UCUHjtOtm5X7pMFSgAL

import casadi.*
x=SX.sym('x')
obj = x^2 + 6*x+13
g = [];
p = [];

nlp_prob = struct('f',obj, 'x',x, 'g',g,'p',p)

opts = struct;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0; %0,3
opts.print_time = 0; %0,1
opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
opts.ipopt.acceptable_obj_change_tol = 1e-6; 

solver = nlpsol('solver', 'ipopt', nlp_prob,opts)

args = struct;
args.lbx = -inf;  % unconstrained optimization 
args.ubx = inf;   % unconstrained optimization
args.lbg = -inf;  % unconstrained optimization
args.ubg = inf;   % unconstrained optimization

args.p   =  [];  % There are no parameters in this optimization problem
args.x0  = -0.5; % initialization of the optimization problem

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
x_sol = full(sol.x)            % Get the solution
min_value = full(sol.f)   % Get the value function