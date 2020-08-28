function m = RRPlanarModel()

    model.l1  = 0.2; % length of link 1
    model.l2  = 0.2; % length of link 2
    model.lc1 = 0.1; % COM of link 1
    model.lc2 = 0.1; % COM of link 2
    model.m1  = 0.5; % mass of link 1
    model.m2  = 0.5; % mass of link 2
    model.I1  = 0.5 * 0.1*0.1; % moment of inertia of link 1
    model.I2  = 0.5 * 0.1*0.1; % moment of inertia of link 2
    model.b1  = 0; % dumper of joint 1
    model.b2  = 0; % dumper of joint 2
    %
    % disturbance
    %
    model.wm = 1; % disturbance in M term, 1 => no disturbance
    model.wc = 1; % disturbance in C term, 1 => no disturbance
    model.wg = 1; % disturbance in G term, 1 => no disturbance
    %
    % environment parameters
    model.g = 9.81;

    m = model;

end