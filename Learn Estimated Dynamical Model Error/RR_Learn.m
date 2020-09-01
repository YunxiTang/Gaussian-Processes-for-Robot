%%% to learn the inverse dynamics error of RR robot
clear; clear global; close all;close all;
randn('seed',1);
% store the collected data
Memory_X = [];
Yc1 = [];
Yc2 = [];
CAPACITY = 500;
F_GP = [];
SIG_1 = [];
SIG_2 = [];
MU_1 = [];
MU_2 = [];

dt = 0.01;

% import robot model
model = RRPlanarModel();

% set the dynamics uncertainty
model.b1 = 0.5; 
model.b2 = 0.5; 
model.wm = 1.8;
model.wc = 1.5; 
model.wg = 1.2;

% initialize the robot configuration
x0 = [0.0; 0.0; deg2rad(0); deg2rad(0)];
tspan = [0 50];
t_grid = linspace(tspan(1),tspan(2),(tspan(2)-tspan(1))/dt);

% initialize the GP model (n GP Models)
% meanfunc1 =  {@meanLinear};      % empty: dont use a mean function
meanfunc1 = [];
covfunc1 = @covSEiso; % Squared Exponential covriance matrix
% covfunc1 = {@covPERiso,{@covSEiso}};
likfunc1 = @likGauss; % Gaussian likelihood

% meanfunc2 = {@meanLinear};       % empty: dont use a mean function
meanfunc2 = [];
covfunc2 = @covSEiso; % Squared Exponential covriance matrix
% covfunc2 ={@covPERiso,{@covSEiso}};
likfunc2 = @likGauss; % Gaussian likelihood

% initialize the hyperparameter struct
hyp1 = struct('mean',[],'cov',[1;1], 'lik', -1);
hyp2 = struct('mean',[],'cov',[1;1], 'lik', -1);
hyp1_min = hyp1;
hyp2_min = hyp2;
%% initial state
x = x0;
mu = zeros(4,1);
sigma = 0.1 * diag([0.5,0.5,1,1]);
%% LEARN
% ode options
options = odeset('RelTol',1e-8, 'AbsTol',1e-8);
xargu = [0;0;0;0;0;0;x(1);x(2);x(3);x(4)];
Memory_X = [Memory_X xargu];
Yc1 = [Yc1 0];
Yc2 = [Yc2 0];
for i=1:length(t_grid)
    tic
    t_current = t_grid(i);
    
    [qr,qrd,qrdd] = desiredJointTrajectory(t_current);
    
    % form of the collected data (column vector): x = [qrdd;qrd;q;qd] 
    query_x = [qrdd(1);qrdd(2);qrd(1);qrd(2);qr(1);qr(2);
               x(1);x(2);x(3);x(4)];
  
    % prediction
    [y1_mu, y1_s2, f1_mu, f1_s2] = gp(hyp1_min,@infGaussLik,meanfunc1,covfunc1,likfunc1,Memory_X',Yc1',query_x');
    [y2_mu, y2_s2, f2_mu, f2_s2] = gp(hyp2_min,@infGaussLik,meanfunc2,covfunc2,likfunc2,Memory_X',Yc2',query_x');
    u = controlLaw_ComputedTorque(t_current,model,x,[],[1],[1]) + [f1_mu;f2_mu];
    SIG_1 = [SIG_1 sqrt(f1_s2)];
    SIG_2 = [SIG_2 sqrt(f2_s2)];
    MU_1 = [MU_1 f1_mu];
    MU_2 = [MU_2 f2_mu];
    % GET NEXT STATE
    t_s = [t_current t_current+dt];
    
    sol = ode45(@(t,x)(RRPlanarDynamics(model,x,u)), t_s, x, options);
    
    t_s_grid = linspace(t_s(1),t_s(2),5);
    
    x_s = deval(sol,t_s_grid);
    
    x_next = x_s(:,5);% + mvnrnd(mu,sigma)';
    
    X(:,i) = x_next;
    
    % ESTIMATED MODEL DYNAMICS
    tau_hat = controlLaw_ComputedTorque(t_current,model,x,x_next,[],[]);
    
    % ERROR OF DYNAMICAL MODEL
    delta_tau = -tau_hat + u;
    
    F_GP(:,i) = delta_tau;
    
    %
    Xr(:,i) = [qr(1);qr(2);qrd(1);qrd(2)];
    
    data_point_y = delta_tau;% + 0.01*gpml_randn(0.02,size(delta_tau'));
    data_point_x = [qrdd(1);qrdd(2);qrd(1);qrd(2);qr(1);qr(2);
                    x_next(1);x_next(2);x_next(3);x_next(4)];
                
    [D, n] = size(Memory_X);
    if n <= CAPACITY
        disp('Not Enough Data!!! Collecting...');
        Memory_X = [Memory_X data_point_x];  % insert data directly
        Yc1 = [Yc1 data_point_y(1)];
        Yc2 = [Yc2 data_point_y(2)];
    elseif mod(i,20) == 0
        [Memory_X, indice] = randn_delete(Memory_X,'col');   % delete a column randnly
        Memory_X = [Memory_X data_point_x];  % insert the new coming data
        
        Yc1 = certain_delete(Yc1,indice,'col');
        Yc2 = certain_delete(Yc2,indice,'col');
        Yc1 = [Yc1 data_point_y(1)];
        Yc2 = [Yc2 data_point_y(2)];
    end
    disp('-------------NEXT STATE!!!------------------');
    disp(t_current);
    % training ...
    if mod(i,20) == 0
        hyp1_min = minimize(hyp1,@gp,-5,@infGaussLik,meanfunc1,covfunc1,likfunc1,Memory_X',Yc1');
        hyp2_min = minimize(hyp2,@gp,-5,@infGaussLik,meanfunc2,covfunc2,likfunc2,Memory_X',Yc2');
    end
    % update x
    x = x_next;
    toc
end
    
    
%%
e1 = (Xr(1,:)-X(1,:));
e2 = (Xr(2,:)-X(2,:));
figure(1);
subplot(2,1,1);
plot(t_grid,X(1,:),'-.',t_grid,Xr(1,:),'linewidth',2.0);
legend('q_1','q_{1d}');
subplot(2,1,2);
plot(t_grid,X(2,:),'-.',t_grid,Xr(2,:),'linewidth',2.0);
legend('q_2','q_{2d}');

figure(2);
subplot(2,1,1);
plot(t_grid,e1,'linewidth',2.0);
title('Error_{q1}');
grid on;
subplot(2,1,2);
plot(t_grid,e2,'linewidth',2.0);
title('Error_{q2}');
grid on;

figure(3);
plot(t_grid,F_GP,'-.','linewidth',2.0);
legend('\tau_{1_{GP}}','\tau_{2_{GP}}');
grid on;

RMSE_1 = sqrt((e1 * e1') / length(t_grid));
RMSE_2 = sqrt((e2 * e2') / length(t_grid));

%% 
figure(4);
subplot(2,1,1);
plot(t_grid,MU_1,'linewidth',2.0);hold on;plot(t_grid,MU_1-3*SIG_1,'b-.',t_grid,MU_1+3*SIG_1,'b-.');
subplot(2,1,2);
plot(t_grid,MU_2,'linewidth',2.0);hold on;plot(t_grid,MU_2-3*SIG_2,'b-.',t_grid,MU_2+3*SIG_2,'b-.');
