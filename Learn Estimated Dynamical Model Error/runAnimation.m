function runAnimation(model, tspan, t, x, x_dsr)

    %%%% Showing the two link planar
    %
    % For easier to view the initial pose, it will pause a couple of secs.
    %
    clf;
    hold on;
    title(sprintf('t = %6.4f',0));
    if ~isempty(x_dsr)
        plot(x_dsr(1,:), x_dsr(2,:), 'g')
    end
    drawRRPlanar(0, x(:,1), model);
    hold off;
    pause(2);

    % Animation
    %
    sim_time = 0;
    tic; % Start stopwatch timer for simulation

    while sim_time < tspan(end)

        % Interpolate to get the new point at current time:
        sim_time = toc;
        xq_ode = interp1(t', x', sim_time, 'spline')';

        clf;
        hold on;
        title(sprintf('t = %6.4f',sim_time));
        %
        if ~isempty(x_dsr)
            plot(x_dsr(1,:), x_dsr(2,:), 'g')
        end
        %
        drawRRPlanar(sim_time, xq_ode, model);
        hold off;

        drawnow;

        % Update simulation time
        sim_time = toc;
    end

end