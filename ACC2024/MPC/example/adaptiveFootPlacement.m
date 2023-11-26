clear
clc

addpath('../../../casadi-3/')

%% Example of using Casadi
% https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi
% https://www.youtube.com/playlist?list=PLK8squHT_Uzej3UCUHjtOtm5X7pMFSgAL

import casadi.*

g=9.8;
h=0.231; % COM height
omega=sqrt(g/h);

delta_transformation = @(stepDuration)(exp(-omega*stepDuration));
delta_inverse_transformation = @(delta)(log(delta)/(-omega));

% Steady state periodic walking 
% steady state DCM offset b = xi - z
b_steady=0.06;
stepDuration_steady = 0.2; % second
delta_t_steady = delta_transformation(stepDuration_steady);
stepLength_steady = b_steady/delta_t_steady - b_steady; % meter

% Constraints
stepLength_max = 0.18; % m
stepLength_min = 0;
stepDuration_min = 0.1; % s
stepDuration_max = 0.3;
% delta_transformation is decreasing function
delta_t_max = delta_transformation(stepDuration_min);
delta_t_min = delta_transformation(stepDuration_max);

% Initial value of DCM offset
b_initial=0.1;
% Decision variables
b1 = SX.sym('b1');
s1 = SX.sym('s1');
delta_t1 = SX.sym('delta_t1'); 

b2 = SX.sym('b2');
s2 = SX.sym('s2');
delta_t2 = SX.sym('delta_t2');

b3 = SX.sym('b3');
s3 = SX.sym('s3');
delta_t3 = SX.sym('delta_t3');

obj = (b1-b_steady)^2 + (b2-b_steady)^2 + (b3-b_steady)^2
% Equality Constraint
% b = delta_t1*(s1+b1); 
g = [delta_t1*(s1+b1)-b_initial; 
     delta_t2*(s2+b2)-b1; 
     delta_t3*(s3+b3)-b2]

p = []
nlp_prob = struct('f',obj, 'x',[b1, b2, b3, s1, s2, s3, delta_t1, delta_t2, delta_t3], 'g',g,'p',p)
opts = struct;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0; %0,3
opts.print_time = 0; %0,1
opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
opts.ipopt.acceptable_obj_change_tol = 1e-6; 

solver = nlpsol('solver', 'ipopt', nlp_prob,opts)

args = struct;
args.lbx = [-1, -1, b_steady, stepLength_min, stepLength_min, stepLength_min, delta_t_min, delta_t_min, delta_t_min];
args.ubx = [ 1,  1,  b_steady, stepLength_max, stepLength_max, stepLength_max, delta_t_max, delta_t_max, delta_t_max];
args.lbg = [0,0,0];
args.ubg = [0,0,0];

args.p   =  [];  % There are no parameters in this optimization problem
args.x0  = [-1, -1, -1, 0, 0, 0, delta_t_min, delta_t_min, delta_t_min]; % initialization of the optimization problem
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
x_sol = full(sol.x)            % Get the solution
t1 = delta_inverse_transformation(x_sol(7))
t2 = delta_inverse_transformation(x_sol(8))
t3 = delta_inverse_transformation(x_sol(9))

min_value = full(sol.f)   % Get the value function

%% Simulate the foot steps

z_ini=0;
xi_u_ini = z_ini + b_initial;

