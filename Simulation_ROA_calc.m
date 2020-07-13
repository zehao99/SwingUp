clc;
clear;

% Put into state space form
A = [0 1 0      0; ...
     0 0 1.089  0;...
     0 0 0      1; ...
     0 0 13.067 0];
B = [0;0.7407;0;1.111];
C = eye(4);
D = 0;
sys = ss(A,B,C,D);

% Check controllability
if rank(ctrb(sys))==4
    disp('System is controllable!');
else
    disp('System is not controllable.');
end

% LQR Controller
Q = [5 0 0 0; 0 1 0 0; 0 0 5 0; 0 0 0 1]; %cost on states
R = 10; %cost on inputs
[K,S,E] = lqr(sys,Q,R);

%Simulation Parameters
theta_initial = 45;
dtheta_initial = 0;

% Run Simulation
sim('Simulation.slx');

%% Basin of Attraction Analysis
numThetas = 50;
numDThetas = 50;
theta_initials = linspace(0,90,numThetas+1);
dtheta_initials = linspace(0,500,numDThetas+1);
counter = 0;
dataMatrix = [0 0 1];
for i=1:numThetas
    for j=1:numDThetas
        counter = counter + 1;
        theta_initial = theta_initials(i); %[deg]
        dtheta_initial = dtheta_initials(j); %[deg/s]
        tic;
        sim('ROA.slx');        
        if (isempty(endTime.Time))
            success = 1;
        else
            success = 0;
        end
        dataMatrix = [dataMatrix; theta_initial dtheta_initial success];
        disp('Iteration: ');
        disp(counter);
    end
end

dataMatrix = load('RegionOfAttraction.mat');
dataMatrix = dataMatrix.dataMatrix;


%% Fill out results
dataMatrixFull = [dataMatrix; ...
    [-dataMatrix(:,1) dataMatrix(:,2) dataMatrix(:,3)];...
    [dataMatrix(:,1) -dataMatrix(:,2) dataMatrix(:,3)];...
    [-dataMatrix(:,1) -dataMatrix(:,2) dataMatrix(:,3)]];

%% Plot Results
figure(1);
hold on;
for i=1:length(dataMatrixFull)
    if (dataMatrixFull(i,3)==1)
        plot(dataMatrixFull(i,1)/180 * pi, dataMatrixFull(i,2)/180 * pi, 'b.');
    end
end
hold off;
xlabel('theta [rad]');
ylabel('dtheta [rad/s]');
title('Region of Attraction');
