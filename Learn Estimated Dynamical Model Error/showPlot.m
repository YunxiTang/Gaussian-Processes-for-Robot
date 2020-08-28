function showPlot(t, x, x_dsr,model,controlLaw)

    %%%% Show plot - Trajectory
    figure(20);
    hold on;
    plot(t, rad2deg(x(1,:)), '-r', 'LineWidth',2);
    plot(t, rad2deg(x(2,:)), '-b', 'LineWidth',2);
    plot(t, rad2deg(x_dsr(1,:)), ':r', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(2,:)), ':b', 'LineWidth',1);
    xlabel('t');
    ylabel('Joint angle (degree)');
    title('Joint Angle Trajectory');
    legend('q_1', 'q_2','desired q_1', 'desired q_2','FontSize',12,...
        'FontName','Times Newroman');
    grid on;
    hold off;

    %%%% Show plot - Velocity
    figure(21);
    hold on;
    plot(t, rad2deg(x(3,:)), '-r', 'LineWidth',2);
    plot(t, rad2deg(x(4,:)), '-b', 'LineWidth',2);
    plot(t, rad2deg(x_dsr(3,:)), ':r', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(4,:)), ':b', 'LineWidth',1);
    xlabel('t');
    ylabel('angular velocity (degree/s)');
    title('Joint Angular Velocity');
    legend('q_1', 'q_2', 'desired q_1', 'desired q_2','FontSize',12,...
        'FontName','Times Newroman');
    grid on;
    hold off;
    
    %%%% Show errors of angle
    figure(2);
    hold on;
    subplot(2,1,1);
    plot(t,rad2deg(x_dsr(1,:)-x(1,:)),'-r', 'LineWidth',2);
    xlabel('t(s)');
    ylabel('angle error(degree)');
    grid on;
    title('Joint 1 Angle Error');
    subplot(2,1,2);
    plot(t,rad2deg(x_dsr(2,:)-x(2,:)),'-b', 'LineWidth',2);
    xlabel('t(s)');
    ylabel('angle error(degree)');
    title('Joint 2 Angle Error');
    grid on;
    hold off;
    
    %%%% Show angular errors 
    figure(22);
    hold on;
    subplot(2,1,1);
    plot(t,rad2deg(x_dsr(3,:)-x(3,:)),'-r', 'LineWidth',2);
    xlabel('t(s)');
    ylabel('angluar velocity error(degree/s)');
    grid on;
    title('Joint 1 Angular Velocity Error');
    subplot(2,1,2);
    plot(t,rad2deg(x_dsr(4,:)-x(4,:)),'-b', 'LineWidth',2);
    xlabel('t(s)');
    ylabel('angluar velocity error(degree/s)');
    title('Joint 2 Angular Velocity Error');
    grid on;
    hold off;
   
    if strcmpi(controlLaw,'JacobianTranspose')
        %%% operation space trajectory
        [x_dsr,~]=desiredOperTrajectory(t);
    
        %%% real operation trajectory
        figure;
        hold on;
        [~,p2]=RRPlanarFKine(model,x);
        plot(x_dsr(1,:),x_dsr(2,:),'-',p2(1,:),p2(2,:),'-.','LineWidth',2);
        grid on;
        title('Trajectory in  Cartesian plane');
        legend('Desired Trajectory','Actual Trajectory');
        axis equal;
        hold off;
    
        %%% operational errors
        figure;
        hold on;
        subplot(3,2,1);
        plot(t,x_dsr(1,:),'-',t,p2(1,:),'-.','LineWidth',2);
        grid on;
        title('x trajectory vs. time');
        legend('Desired x trajectory','Actual x trajectory');
    
        subplot(3,2,2);
        plot(t,x_dsr(1,:)-p2(1,:),'LineWidth',2);
        grid on;
        title('X trajectory error');
    
        subplot(3,2,3);
        plot(t,x_dsr(2,:),'-',t,p2(2,:),'-.','LineWidth',2);
        grid on;
        title('y trajectory vs. time');
        legend('Desired y trajectory','Actual y trajectory');

        subplot(3,2,4);
        plot(t,x_dsr(2,:)-p2(2,:),'LineWidth',2);
        grid on;
        title('Y trajectory error');
        hold off;
    
        subplot(3,2,5:6);
        plot(t,sqrt((x_dsr(1,:)-p2(1,:)).^2+(x_dsr(2,:)-p2(2,:)).^2),'LineWidth',2);
        grid on;
        title('Distance Error');
    else
        
    end
    
end