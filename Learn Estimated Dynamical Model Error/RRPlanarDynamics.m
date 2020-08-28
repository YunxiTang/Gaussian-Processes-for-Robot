function xd = RRPlanarDynamics(model, x, u)
%
% INPUTS:
%    model: struct
%    x: [4,1] = [q1; q2; q1d; q2d]
%    u: [2,1] = input torque
%
% OUTPUTS:
%    xd: [4,1] = [q1d; q2d; q1dd; q2dd]

    q  = x(1:2,:);
    qd = x(3:4,:);

    [M,C,G,F] = RRPlanarManipulatorEquation(model, x);
    
    % NOTES:
    % xd  = [qd; qdd];
    % qdd = inv(M(q))*(u-V(q,qd)*qd - G(q))
    %
    xd = [qd; M \ (u -C*qd - F*qd - G)];
    
end