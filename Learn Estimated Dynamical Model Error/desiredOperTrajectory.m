function [x,xd] = desiredOperTrajectory(t)
%DESIREDOPERTRAJECTORY generate the desired operation space trajectory
%   x = [x_oper, y_oper]
%   xd = [vx_oper,vy_oper]
    x_dsr = 1 + 0.4*cos(2*pi*0.5*t-2);
    vx_dsr = -0.4*2*pi*0.5*sin(2*pi*0.5*t-2); 
    
    y_dsr = 1 + 0.4*sin(2*pi*0.5*t-2);
    vy_dsr = 0.4*2*pi*0.5*cos(2*pi*0.5*t-2);
    x = [x_dsr;
         y_dsr];
    xd = [vx_dsr;
          vy_dsr];
end

