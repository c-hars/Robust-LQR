function K = dlqr_multiplant(A_set, B_set, Q, R)
% Guaranteed-cost state feedback over a plant set. Sign convention matches dlqr(): u = -Kx.

    n_plants = length(A_set);
    n = size(Q,1);
    m = size(R,1);
    assert (size(A_set{1}, 1) == n);
    assert (size(B_set{1}, 2) == m);
    for i=2:n_plants
        assert (all(size(A_set{i}) == size(A_set{i-1})))
        assert (all(size(B_set{i}) == size(B_set{i-1})))
    end

    n_verts = n_plants;
    A_verts = A_set;
    B_verts = B_set;
    
    pd_defn = 1e-12;
    
    % Decision variables
    yalmip('clear')
    X = sdpvar(n, n, 'full');
    L = sdpvar(m, n, 'full');
    mu = sdpvar(1);
    P_set = cell(1, n_verts);
    W_set = cell(1, n_verts);
    Constraints = [mu >= 0];
    
    Q_sqrt = sqrtm(Q);
    R_sqrt = sqrtm(R);
    B_w = eye(n);
    C_z = [Q_sqrt; zeros(m,n)];
    D_zu = [zeros(n,m); R_sqrt];

    for i = 1:n_verts
        A = A_verts{i};
        B = B_verts{i};
    
        P_set{i} = sdpvar(n, n, 'symmetric');      P = P_set{i};
        W_set{i} = sdpvar(n+m, n+m, 'symmetric');  W = W_set{i};
    
        % Stability + gramian
        stab_LMI = [P,            A*X + B*L,     B_w; ...
                    (A*X + B*L)', X + X' - P,    zeros(n);  ...
                    B_w',         zeros(n),      eye(n)   ];
        Constraints = [Constraints, stab_LMI >= pd_defn];
    
        % H2 performance
        perf_LMI = [W,                 C_z*X + D_zu*L;
                    (C_z*X + D_zu*L)', X + X' - P     ];
        Constraints = [Constraints, perf_LMI >= pd_defn];
    
        % trace(W) <= mu
        Constraints = [Constraints, trace(W) <= mu];
    
    end
    
    % Objective
    Objective = mu;
    options = sdpsettings('solver', 'mosek', 'verbose', 0);
    sol = optimize(Constraints, Objective, options);
    
    if sol.problem == 0
        % disp('Feasible solution found.')
        X = value(X);
        L = value(L);
        K = -L / X;
    else
        % disp('Problem during optimization:');
        error('dlqr_multiplant: %s', sol.info)
    end

end