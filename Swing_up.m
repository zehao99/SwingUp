clc;
clear;

%Add Necessary Files to Path
currentPath = path();
addpath((currentPath));

%Put into state space form
A = [0 1 0      0; ...
     0 0 1.7818  0;...
     0 0 0      1; ...
     0 0 10.69 0];
B = [0;0.9848;0;0.9090];
C = eye(4);
D = 0;
sys = ss(A,B,C,D);

%Check controllability
if rank(ctrb(sys))==4
    disp('System is controllable!');
else
    disp('System is not controllable.');
end

%LQR Controller
Q = [5 0 0 0; 0 1 0 0; 0 0 5 0; 0 0 0 1]; %cost on states
R = 10; %cost on inputs
[K,S,E] = lqr(sys,Q,R);
% Trajectory Optimization

%Setup
p.m1 = 1.0;  % (kg) Cart mass
p.m2 = 0.2;  % (kg) pole mass
p.g = -9.8;  % (m/s^2) gravity
p.l = 1;   % (m) pendulum (pole) length

dist = 1.5;  %How far must the cart translate during its swing-up
maxForce = 200;  %Maximum actuator forces
duration = 1.9;

problem.func.dynamics = @(t,x,u)( cartPoleDynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( u.^2 );  %Input-squared cost function

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = [dist;pi;0;0];
problem.bounds.initialState.upp = [dist;pi;0;0];
problem.bounds.finalState.low = zeros(4,1);
problem.bounds.finalState.upp = zeros(4,1);

problem.bounds.state.low = [-2*dist;-2*pi;-inf;-inf];
problem.bounds.state.upp = [2*dist;2*pi;inf;inf];

problem.bounds.control.low = -maxForce;
problem.bounds.control.upp = maxForce;

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0];

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e5);

problem.options.method = 'hermiteSimpson';

%Solve
soln = optimTraj(problem);

% Unpack the solution
t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);
trajectory = [t' u'];

%Run Simulation
sim('Swingup.slx');







