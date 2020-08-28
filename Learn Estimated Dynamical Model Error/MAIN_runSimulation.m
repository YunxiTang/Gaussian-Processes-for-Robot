clear; clear global; close all;close all;

%% Two Link RR Planar model & enviroment parameters
model = RRPlanarModel();
% For the damping terms, to create some noticable decceleration effects, 
model.b1 = 0.5; 
model.b2 = 0.5; 
model.wm = 1.8;
model.wc = 1.5; 
model.wg = 1.2;

%% Define inital state
x0 = [0; 0; deg2rad(0); deg2rad(0)];
%% Define time span for simulation 
tspan = [0 25];

%% Control law switching
% controlLaw = 'Passive';
% controlLaw = 'PD';
controlLaw = 'ComputedTorque';
%controlLaw = 'JacobianTranspose';
% controlLaw = 'random';


%% Run simulation
[t,x] = runRRPlanarSim(model, x0, controlLaw, tspan);

%% Run animation
% runAnimation(model, tspan, t, x, []);

%% Show plot
[q, qd, qdd] = desiredJointTrajectory(t);
showPlot(t,x,[q;qd;qdd],model,controlLaw);
%%
e1 = (q(1,:) - x(1,:));
e2 = (q(2,:) - x(2,:));
RMSE_1 = sqrt((e1 * e1') / length(t))
RMSE_2 = sqrt((e2 * e2') / length(t))
figure(2);
subplot(2,1,1);
plot(t,e1,'-.','linewidth',2.0);
grid on;
subplot(2,1,2);
plot(t,e2,'linewidth',2.0);
grid on;