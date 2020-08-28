%%% GPR
u = controlLaw_PD(t,model,x);
x_argu = [];
q1_argu = [];
q2_argu = [];
q1d_argu = [];
q2d_argu = [];
x_test = [];
t_train = [];
for k=1:length(t)/2-1
    x_argu(k,:) = [x(:,k); u(:,k)];
    q1_argu(k,:) = x(1,k+1) + 0.02*randn(1,1);
    q2_argu(k,:) = x(2,k+1) + 0.02*randn(1,1);
    q1d_argu(k,:) = x(3,k+1) + 0.05*randn(1,1);
    q2d_argu(k,:) = x(4,k+1) + 0.05*randn(1,1);
end

for i=length(t)/2:length(t)
    x_test(i,:)=[x(:,i); u(:,i)];
end
%% plotter
n = length(q1_argu);
gprMd_q1 = fitrgp(x_argu, q1_argu,'KernelFunction','squaredexponential');
gprMd_q2 = fitrgp(x_argu, q2_argu,'KernelFunction','squaredexponential');
gprMd_q1d = fitrgp(x_argu, q1d_argu,'KernelFunction','squaredexponential');
gprMd_q2d = fitrgp(x_argu, q2d_argu,'KernelFunction','squaredexponential');

q1pred = resubPredict(gprMd_q1);
q1_validate = predict(gprMd_q1, x_test);

q2pred = resubPredict(gprMd_q2);
q2_validate = predict(gprMd_q2, x_test);

q1dpred = resubPredict(gprMd_q1d);
q1d_validate = predict(gprMd_q1d, x_test);

q2dpred = resubPredict(gprMd_q2d);
q2d_validate = predict(gprMd_q2d, x_test);

subplot(2,2,1);
plot(t(1:n),q1_argu,'black.');     
hold on;
plot(t(1:n), q1pred,'green', 'linewidth',3);
hold on;
plot(t, q1_validate,'blue-..','linewidth',2);
hold on;
plot(t,x(1,:),'r-.','linewidth',2);
legend('Observed Targets','Predition from training data','Predition from all data','Groud Truth');
grid on;

subplot(2,2,2);
plot(t(1:n),q2_argu,'black.');     
hold on;
plot(t(1:n), q2pred,'green', 'linewidth',3);
hold on;
plot(t, q2_validate,'blue-..','linewidth',2);
hold on;
plot(t,x(2,:),'r-.','linewidth',2);
legend('Observed Targets','Predition from training data','Predition from all data','Groud Truth');
grid on;


subplot(2,2,3);
plot(t(1:n),q1d_argu,'black.');     
hold on;
plot(t(1:n), q1dpred,'green', 'linewidth',3);
hold on;
plot(t, q1d_validate,'blue-..','linewidth',2);
hold on;
plot(t,x(3,:),'r-.','linewidth',2);
legend('Observed Targets','Predition from training data','Predition from all data','Groud Truth');
grid on;


subplot(2,2,4);
plot(t(1:n),q2d_argu,'black.');     
hold on;
plot(t(1:n), q2dpred,'green', 'linewidth',3);
hold on;
plot(t, q2d_validate,'blue-..','linewidth',2);
hold on;
plot(t,x(4,:),'r-.','linewidth',2);
legend('Observed Targets','Predition from training data','Predition from all data','Groud Truth');
grid on;

