function tau = controlLaw_JacobianTranspose(t,model,x)

    q  = x(1:2,:);
    qd = x(3:4,:);
    
    % Jacobian matrix
    J = RRPlanarJacobian(model,q);
    [~,pos_tip] = RRPlanarFKine(model,q);
    v_tip = J * qd;
    
    %generate the disred trajectory
    [x_dsr,xd_dsr] = desiredOperTrajectory(t);
    [~,~,G,~]=RRPlanarManipulatorEquation(model, x);
    % put own algorithm here, tau will be the output of your controller
    e = x_dsr - pos_tip;
    ed = xd_dsr - v_tip;
    
    Kp = [100000 0;
          0 100000];
    Kd = [10000 0;
          0 10000];
    tau = J'*(Kp*e + Kd*ed)+G;

end