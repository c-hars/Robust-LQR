function K = get_H2_optimal_SF_controller_multiplant(A_set, B_set, Q, R)

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
    
    pd_defn = 1e-3;
    
    % Decision variables
    yalmip('clear')
    X = sdpvar(n, n);
    L = sdpvar(m, n, 'full');
    mu = sdpvar(1);
    P = cell(1, n_verts);
    W = cell(1, n_verts);
    Constraints = [X >= pd_defn*eye(n)];
    
    Q_sqrt = chol(Q, 'lower');
    R_sqrt = chol(R, 'lower');
    C_z = [Q_sqrt; zeros(m,n)];
    D_zu = [zeros(n,m); R_sqrt];
    
    for i = 1:n_verts
        A = A_verts{i};
        B = B_verts{i};
    
        P{i} = sdpvar(n, n);
        W{i} = sdpvar(n+m, n+m);
    
        % Stability constraint
        stab_LMI = [P{i},         A*X + B*L;
                    (A*X + B*L)', X + X' - P{i}];
        Constraints = [Constraints, stab_LMI >= pd_defn*eye(2*n)];
    
        % H2 performance constraint
        CzX_DzuL = C_z*X + D_zu*L;
        perf_LMI = [W{i},       CzX_DzuL;
                    CzX_DzuL',  X + X' - P{i}];
        Constraints = [Constraints, perf_LMI >= pd_defn*eye(2*n + m)];
    
        % trace(W) <= mu
        Constraints = [Constraints, mu >= trace(W{i})];
    end
    
    % Objective
    Objective = mu;
    options = sdpsettings('solver', 'mosek', 'verbose', 0);
    sol = optimize(Constraints, Objective, options);
    
    if sol.problem == 0
        % disp('Feasible solution found.')
        X = value(X);
        L = value(L);
        K = L * inv(X);
    else
        % disp('Problem during optimization:');
        sol.info
    end

end