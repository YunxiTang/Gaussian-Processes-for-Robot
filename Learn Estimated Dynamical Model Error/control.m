function u = control(t,model,x,controlLaw)

    switch controlLaw
        case 'Passive'
            u = controlLaw_Passive(t,model,x);
            
        case 'PD'
            u = controlLaw_PD(t,model,x);
            
        case 'ComputedTorque'
            u = controlLaw_ComputedTorque(t,model,x,[],[1],[1]);

        case 'JacobianTranspose'
            u = controlLaw_JacobianTranspose(t,model,x);
            
        case 'random'
            u = 5 * normrnd(0,20);
            
        otherwise
            u = [0; 0];
    end
    disp(t)
end