function [q, qd, qdd] = desiredJointTrajectory(t)
    q   = [-(pi/2)*sin(t); -(pi/2)*cos(t)];
    qd  = [-(pi/2)*cos(t); (pi/2)*sin(t)];
    qdd = [(pi/2)*sin(t); (pi/2)*cos(t)];
%     q   = [0.5*t.*t; 0.5*t.*t];
%     qd  = [t; t];
%     qdd = [1; 1];
end