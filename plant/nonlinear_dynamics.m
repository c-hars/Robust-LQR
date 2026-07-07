function dxdt = nonlinear_dynamics(x,u,qp)

    % Extract angles and velocities
    v_c_x   = x(4);
    v_c_y   = x(5);
    v_c_z   = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    omega_x = x(10);
    omega_y = x(11);
    omega_z = x(12);

    % Convert delta_i to resulting thrust and torques
    
    M_mix_NL = [ ...
        qp.kF * qp.enabled; ...
        qp.kF * qp.enabled .* qp.y; ...
       -qp.kF * qp.enabled .* qp.x; ...
       -qp.kM * qp.enabled .* qp.dirs; ...
    ];

    abs_angvels = u + qp.nominal_omegas';
    abs_angvels = max(abs_angvels, 0);
    abs_angvels = min(abs_angvels, qp.max_RPM*2*pi/60); % Enforce motor RPM limits

    F_t       = M_mix_NL(1,:) * abs_angvels.^2;
    tau_phi   = M_mix_NL(2,:) * abs_angvels.^2;
    tau_theta = M_mix_NL(3,:) * abs_angvels.^2;
    tau_psi   = M_mix_NL(4,:) * abs_angvels.^2;
    T_c = [tau_phi; tau_theta; tau_psi];
    
    % Nonlinear translation dynamics
    omega = [omega_x; omega_y; omega_z];
    v_c = [v_c_x; v_c_y; v_c_z];
    C_bI = C_x(phi) * C_y(theta) * C_z(psi); % DCM from inertial frame to body frame
    C_Ib = C_bI';
    g = 9.81;
    v_c_dot = -S(omega)*v_c + (1/qp.m) * ([0;0;F_t] + C_bI*[0; 0; -qp.m*g]);
    
    % Nonlinear Euler dynamics
    J = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
    euler_dot = J * omega;
    
    % Nonlinear rotational dynamics
    I = diag([qp.I_xx qp.I_yy qp.I_zz]);
    omega_dot = -inv(I)*S(omega)*I*omega + inv(I)*T_c;

    % Stack in vector
    dxdt = zeros(12,1);
    dxdt(1:3)   = C_Ib*v_c;       % Position derivatives (as measured in the inertial frame)
    dxdt(4:6)   = v_c_dot;        % Velocity derivatives (as measured in the non-inertial body frame)
    dxdt(7:9)   = euler_dot;      % Euler angle derivatives
    dxdt(10:12) = omega_dot;      % Angular acceleration

end