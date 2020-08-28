function [t, x] = runRRPlanarSim(model, x0, controlLaw, tspan)

    %%%% Control law
    u = @(t,x)(control(t,model,x,controlLaw) );
    %%%% Run simulation with ODE45:
    options = odeset('RelTol',1e-8, 'AbsTol',1e-8);
    sol = ode45(@(t,x)(RRPlanarDynamics(model,x,u(t,x))), tspan, x0, options);
    %
    t = linspace(tspan(1),tspan(2),(tspan(2)-tspan(1))*100);
    x = deval(sol,t);

end