function B = get_B_matrix(qp, enabled, method)

    % First need omega_bar.
    if nargin < 2, enabled = ones(1,6); end
    if nargin < 3, method = 'least_squares'; end
    omega_bar = compute_omega_bar(qp, enabled, method);

    % Can then compute the linear motor mixing matrix:
    %   [delta_F_T, delta_tau_x, delta_tau_y, delta_tau_z] ~= M * [delta_omega_1; delta_omega_2; ...; delta_omega_6]
    M = [ ...
         2*qp.kF*omega_bar          .* enabled; ...
         2*qp.kF*omega_bar.*qp.y    .* enabled; ...
        -2*qp.kF*omega_bar.*qp.x    .* enabled; ...
        -2*qp.kM*omega_bar.*qp.dirs .* enabled; ...
    ];

    B = zeros(12, qp.n_rotors);
    B(6,:)  = (1/qp.m)    * M(1,:);  % z_c_ddot = F_t / m
    B(10,:) = (1/qp.I_xx) * M(2,:);  % ω_x_dot  = τ_x / I_xx
    B(11,:) = (1/qp.I_yy) * M(3,:);  % ω_y_dot  = τ_y / I_yy
    B(12,:) = (1/qp.I_zz) * M(4,:);  % ω_z_dot  = τ_z / I_zz
end