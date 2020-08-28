function J = RRPlanarJacobian(model,q)
    l1 = model.l1; l2 = model.l2;
    q1 = q(1); q2 = q(2);
    %
    J(1,1) = - l2*sin(q1+q2) - l1*sin(q1);
    J(1,2) = - l2*sin(q1+q2);
    
    J(2,1) = l2*cos(q1+q2) + l1*cos(q1);
    J(2,2) = l2*cos(q1+q2);
end
