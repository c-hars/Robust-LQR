function plot_trajectories(t_cell, X_cell)

    assert( all(size(t_cell) == size(X_cell)) )
    N = length(t_cell);

    linestyles = {'-','--',':','-.'};
    cmap = colororder();

    for i=1:N
        t = t_cell{i};
        X = X_cell{i};

        subplot(2,1,1); hold on
        plot(t,X(:,1), 'LineStyle', linestyles{i}, 'Color', cmap(1,:))
        plot(t,X(:,2), 'LineStyle', linestyles{i}, 'Color', cmap(2,:))
        plot(t,X(:,3), 'LineStyle', linestyles{i}, 'Color', cmap(3,:))
        ylabel('Position [m]')
        % legend('x_r^*','y_r^*','z_r^*','x_b','y_b','z_b')
        
        subplot(2,1,2); hold on
        plot(t,X(:,7)*180/pi, 'LineStyle', linestyles{i}, 'Color', cmap(1,:))
        plot(t,X(:,8)*180/pi, 'LineStyle', linestyles{i}, 'Color', cmap(2,:))
        plot(t,X(:,9)*180/pi, 'LineStyle', linestyles{i}, 'Color', cmap(3,:))
        ylabel('Euler angles [deg]')
        % legend('\phi_r^*','\theta_r^*','\psi_r^*','\phi_b','\theta_b','\psi_b')
        
        xlabel('Time [s]')
    end

end