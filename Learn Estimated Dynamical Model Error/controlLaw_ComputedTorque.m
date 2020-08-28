function tau = controlLaw_ComputedTorque(t,model,x,x_next, Kp, Kd)
    q  = x(1:2,:);
    qd = x(3:4,:);
    
    % get terms:hatM,hatC,hatG
    [M,C,G,F] = RRPlanarManipulatorEquation(model,x);
    % add disturbance to M, C, G terms
    wm = model.wm; wc = model.wc; wg = model.wg;
    hatM = M.*wm;
    hatC = C.*wc;
    hatG = G.*wg;
    hatF = F;
    if  isempty(Kp) || isempty(Kd)
        % this is just for compute estimated dynamical model
        % put own algorithm here, tau will be the output of your controller
        qdd = x_next(3:4,:);
        tau = hatM*(qdd)+(hatC+hatF)*qd+hatG;
        
    else
        % this is for control
        % with PD terms
        % generate the desired trajectory
        [qr,qrd,qrdd] = desiredJointTrajectory(t);
        %Tune the Kp and Kd gain matrixes
        Kp = [1 0;
              0 1];
        Kd = [0.5 0;
              0 0.5];
        %calculate errors
        e = qr - q;
        ed = qrd - qd;
        tau = hatM*(qrdd)+(hatC+hatF)*qd+hatG+Kp*e+Kd*ed;
    end
        
end