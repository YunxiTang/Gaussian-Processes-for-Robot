function saveAnimation(model, tspan, t, x, x_dsr)

    figHandle = figure; clf;
    hold on;

    vidObj = VideoWriter('animation.mp4','MPEG-4');
    vidObj.FrameRate = 25;
    vidObj.Quality = 100;
    open(vidObj);

    frame_rate = vidObj.FrameRate;
    frame_num = floor(frame_rate*t(end));
    frame_delay = 1/frame_rate;
    sim_time = 0;

    for i=1:frame_num

        xq = interp1(t', x', sim_time, 'spline')';

        % Draw the image
        clf;
        hold on;
        title(sprintf('ODE45, t = %6.4f',sim_time));
        if ~isempty(x_dsr)
            plot(x_dsr(1,:), x_dsr(2,:), 'g')
        end
        drawRRPlanar(sim_time, xq, model);

        hold off;

        % Write data to video file
        writeVideo(vidObj,getframe(figHandle));

        % time step system to next frame:
        sim_time = sim_time + frame_delay;
    end


    close(vidObj);

end