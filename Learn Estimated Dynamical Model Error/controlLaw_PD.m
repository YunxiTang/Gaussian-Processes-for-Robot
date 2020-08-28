function tau = controlLaw_PD(t,model,x)

    
    % Obtain the current joint position and velocity
    q  = x(1:2,:);
    qd = x(3:4,:);
    
    % Sample code for a PD controller 
    % with gravity compensation for trajetory tracking
    
    % (1) Please define your desiredJointTrajectory(t) 
    %     use the desiredJointTrajectory(t) to generate the desired
    %     trajectory
    [qr,qrd,~] = desiredJointTrajectory(t);
    e = qr - q;
    ed = qrd - qd;
    % (2) Define the diagonal matrix for Kp and Kd, 
    %     choose your own gains 
    Kp = [150 0;
          0 150];
    Kd = [90 0;
          0 70];

    % (3) Get the gravity term using the function
    %     RRPlanarManipulatorEquation(model, x);
    [~,~,G,~]=RRPlanarManipulatorEquation(model, x);
    % (4) create the overall controller and apply to tau 
    tau =(Kp*e+Kd*ed) + G;
end