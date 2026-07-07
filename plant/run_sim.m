function [T,X] = run_sim(qp, tspan, x0, enabled_fcn, ctrl_fcn)
    arguments
        qp
        tspan
        x0
        enabled_fcn
        ctrl_fcn = @(t,x,k) -qp.K * x
    end

    timestep = qp.Ts;
    t = 0;
    x = x0;
    T = t;
    X = x.';
    
    k = 0;
    while t < tspan(end)
        k = k+1;
    
        % Compute discrete-time control input (ZOH for next Ts)
        u = ctrl_fcn(t,x,k);
    
        % Integrate from t to t+Ts, keeping u held constant
        qp.enabled = enabled_fcn(t);
        ode = @(t, x) nonlinear_dynamics(x, u, qp);
        [t_local, x_local] = ode45(ode, [t, t+timestep], x);
    
        % Update state and time
        x = x_local(end, :).';      % terminal state
        t = t + timestep;
    
        % Store trajectory and timesteps
        T = [T; t_local(2:end)];
        X = [X; x_local(2:end, :)];
    end

end