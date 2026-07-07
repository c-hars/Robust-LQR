%% Ball-and-stick hexacopter animation + plots

Ts_render = 1/10;

t = tr;
X = Xr;

t_render = 0:Ts_render:t(end);

x_I = interp1(t, X(:,1), t_render);
y_I = interp1(t, X(:,2), t_render);
z_I = interp1(t, X(:,3), t_render);

vx_I = interp1(t, X(:,4), t_render);
vy_I = interp1(t, X(:,5), t_render);
vz_I = interp1(t, X(:,6), t_render);

phi   = interp1(t, X(:,7), t_render);
theta = interp1(t, X(:,8), t_render);
psi   = interp1(t, X(:,9), t_render);

wx_I = interp1(t, X(:,10), t_render);
wy_I = interp1(t, X(:,11), t_render);
wz_I = interp1(t, X(:,12), t_render);

X_I = [x_I; y_I; z_I; vx_I; vy_I; vz_I; phi; theta; psi; wx_I; wy_I; wz_I]';


% %%

figure(1); clf

% Get motor RPMs
RPM_history = zeros(length(t_render),6);
for k=1:length(t_render)
    u = -qp.K * X_I(k,:)';
    abs_angvels = u + qp.nominal_omegas';
    abs_angvels = abs_angvels .* sqrt(thrust_fcn(t_render(k)))';
    abs_angvels = max(abs_angvels, 0);
    abs_angvels = min(abs_angvels, qp.max_RPM*2*pi/60); % Enforce motor RPM limits
    RPM_history(k,:) = abs_angvels * 60 / (2*pi);
end

% %%

t = tiledlayout(1,4);
t.TileSpacing = 'compact';
t.Padding = 'compact';

% --- Left ---
ax1 = nexttile(1);

% --- Middle ---
middle = tiledlayout(t, 3, 1, TileSpacing='compact', Padding='compact');
middle.Layout.Tile = 2;

ax2 = nexttile(middle, 1);
ax3 = nexttile(middle, 2);
ax4 = nexttile(middle, 3);

% --- Right ---
right = tiledlayout(t, 2, 1, TileSpacing='compact', Padding='compact');
right.Layout.Tile = 3;

ax5 = nexttile(right, 1);
ax6 = nexttile(right, 2);

% --- RightRight ---
rightright = tiledlayout(t, 6, 1, TileSpacing='compact', Padding='compact');
rightright.Layout.Tile = 4;
ax7_cell = cell(1,6);
for i=1:6
    ax7_cell{i} = nexttile(rightright, i);
end


cmap = colororder();



% --- Motor RPMs ---
RPM_history_markers = cell(1,6);
for i=1:6
    hold(ax7_cell{i},'on')
    plot(ax7_cell{i},t_render,RPM_history(:,i), 'Color', cmap(i,:), 'DisplayName', sprintf("Motor %d", i))
    ylim(ax7_cell{i}, [0 qp.max_RPM])
    legend(ax7_cell{i})
    grid(ax7_cell{i}, 'on')
    RPM_history_markers{i} = scatter(ax7_cell{i}, 0, RPM_history(1,i), 36, cmap(i,:), 'filled', 'o', 'HandleVisibility', 'off');
    hold(ax7_cell{i},'off')
end
title(ax7_cell{1}, 'Effective RPMs')
xlabel(ax7_cell{end}, 'Time [s]')

% --- Motor/propeller health ---
hold(ax2,'on')
thrust_history = ones(numel(t_render), 6);
for k=1:length(t_render)
    thrust_history(k,:) = thrust_fcn(t_render(k));
end
thrust_history_markers = cell(1,6);
for i=1:6
    if ~all(thrust_history(:,i) == 1)
        plot(ax2, t_render, 100*thrust_history(:,i), 'Color', cmap(i,:), 'DisplayName', sprintf("Motor %d", i))
        thrust_history_markers{i} = scatter(ax2, 0, 100*thrust_history(1,i), 36, cmap(i,:), 'filled', 'o', 'HandleVisibility', 'off');
    end
end
plot(ax2, t_render, 100*ones(size(t_render)), ':', 'Color', [1 1 1]*0, 'DisplayName', 'Others')
hold(ax2,'off')
legend(ax2)
ylabel(ax2,'Health [%]')
ylim(ax2, [0 110])


% --- xyz ---
xyz_markers = cell(1,3);
hold(ax3,'on')
for i=1:3
    plot(ax3, t_render, X_I(:,i), 'Color', cmap(i,:))
    xyz_markers{i} = scatter(ax3, 0, X_I(1,i), 36, cmap(i,:), 'filled', 'o', 'HandleVisibility', 'off');
end
hold(ax3,'off')
legend(ax3,'x','y','z')
ylabel(ax3,'Position [m]')

% --- Euler angles ---
euler_angle_markers = cell(1,3);
hold(ax4,'on')
for i=1:3
    plot(ax4, t_render, X_I(:,6+i)*180/pi, 'Color', cmap(i,:))
    euler_angle_markers{i} = scatter(ax4, 0, X_I(1,6+i), 36, cmap(i,:), 'filled', 'o', 'HandleVisibility', 'off');
end
hold(ax4,'off')
legend(ax4,'\phi','\theta','\psi')
ylabel(ax4,'Euler angles [deg]')
xlabel(ax4,'Time [s]')

% --- Orientation ---
orientation_plot_arms = cell(1,6);
orientation_plot_armends = cell(1,6);
orientation_plot_txtlabels = cell(1,6);
hold(ax5,'on')
C_bI = C_x(phi(1)) * C_y(theta(1)) * C_z(psi(1));
for i=1:qp.n_rotors
    arm_pos = C_bI' * [qp.x(i); qp.y(i); 0];
    orientation_plot_arms{i} = plot3(ax5, [0 arm_pos(1)], [0 arm_pos(2)], [0 arm_pos(3)], '-', 'Color', [1 1 1]*0.5);
    orientation_plot_armends{i} = scatter3(ax5, arm_pos(1), arm_pos(2), arm_pos(3), 512,'MarkerEdgeColor','k','MarkerFaceColor',cmap(i,:));
    orientation_plot_txtlabels{i} = text(ax5, arm_pos(1), arm_pos(2), sprintf("%d", i), 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
orientation_plot_CoM = scatter3(ax5,0,0,0,2048,'k.');
hold(ax5,'off')
xlabel(ax5, 'x [m]')
ylabel(ax5, 'y [m]')
xlim(ax5, [-qp.l qp.l]*1.6)
ylim(ax5, [-qp.l qp.l]*1.6);
view(ax5, 0, 90)
axis(ax5, 'equal')
grid(ax5, 'on')

hold(ax6,'on')
axes_init = C_bI' * (qp.l * eye(3));
orientation_plot_xaxis = plot3(ax6, [0 axes_init(1,1)], [0 axes_init(2,1)], [0 axes_init(3,1)], 'Color', 'r', 'LineWidth', 2);
orientation_plot_yaxis = plot3(ax6, [0 axes_init(1,2)], [0 axes_init(2,2)], [0 axes_init(3,2)], 'Color', 'g', 'LineWidth', 2);
orientation_plot_zaxis = plot3(ax6, [0 axes_init(1,3)], [0 axes_init(2,3)], [0 axes_init(3,3)], 'Color', 'b', 'LineWidth', 2);
hold(ax6,'off')
% xlabel(ax6, 'x')
% ylabel(ax6, 'y')
% zlabel(ax6, 'z')
xlim(ax6, [-qp.l qp.l]*1.2)
ylim(ax6, [-qp.l qp.l]*1.2);
zlim(ax6, [-qp.l qp.l]*1.2);
view(ax6, 20, 25)
grid(ax6, 'on')

% %% ax1

base_color = [0 0 1];

hold(ax1,'on')
grid(ax1,'on')
view(ax1, 20, 25)
axis(ax1, 'equal')
xlabel(ax1,'x');
ylabel(ax1,'y');
zlabel(ax1,'z')

% --- Add origin marker ---
origin = plot3(ax1,0,0,0,'ko','MarkerFaceColor','k','MarkerSize',12);

% --- Initialize arms ---
arms = gobjects(1,6);
for i = 1:6
    arms(i) = plot3(ax1, [0 qp.l*cos(qp.phi(i))], [0 qp.l*sin(qp.phi(i))], [0 0], ...
        'LineWidth', 4, 'Color', [base_color 0.4]);
end

% --- Initialize CoM ---
body = plot3(ax1, 0,0,0,'o','MarkerFaceColor',base_color,'MarkerSize',12);

hold(ax1,'off')

% --- Animation loop ---
for k = 1:length(t_render)

    C_bI = C_x(phi(k))*C_y(theta(k))*C_z(psi(k));

    t_k = t_render(k);
    available_thrust = thrust_fcn(t_k);
    arm_alphas = 0.1 + 0.9*(available_thrust); % fade transparency when motor is partially/fully disabled

    for i = 1:6
        arm_end = C_bI' * [qp.x(i); qp.y(i); 0];
        set(body, ...
            'XData', [x_I(k) x_I(k)], ...
            'YData', [y_I(k) y_I(k)], ...
            'ZData', [z_I(k) z_I(k)]);
        set(arms(i), ...
            'XData', [x_I(k) x_I(k)] + [0 arm_end(1)], ...
            'YData', [y_I(k) y_I(k)] + [0 arm_end(2)], ...
            'ZData', [z_I(k) z_I(k)] + [0 arm_end(3)]);
        set(arms(i),'Color',[base_color arm_alphas(i)]); % red & semi-transparent
    end
    
    xlim(ax1, [min([x_I y_I]), max([x_I y_I])] + [-1 1]*qp.l)
    ylim(ax1, [min([x_I y_I]), max([x_I y_I])] + [-1 1]*qp.l)
    zlim(ax1, [min(z_I), max(z_I)] + [-1 1]*qp.l)

    % --- ax2 ---
    for i=1:6
        if ~isempty(thrust_history_markers{i})
            thrust_history_markers{i}.XData = t_k;
            thrust_history_markers{i}.YData = available_thrust(i)*100;
        end
    end

    % --- ax3 and ax4 ---
    for i=1:3
        xyz_markers{i}.XData = t_k;
        xyz_markers{i}.YData = X_I(k,i);
        euler_angle_markers{i}.XData = t_k;
        euler_angle_markers{i}.YData = X_I(k,6+i)*180/pi;
    end

    % --- ax7 ---
    for i=1:6
        RPM_history_markers{i}.XData = t_k;
        RPM_history_markers{i}.YData = RPM_history(k,i);
    end

    % --- ax5 ---
    eps = 0.01; % add epsilon to z positions to bring an object to the front
    orientation_plot_CoM.XData = X_I(k,1);
    orientation_plot_CoM.YData = X_I(k,2);
    orientation_plot_CoM.ZData = X_I(k,3) + eps; 
    copter_pos = X_I(k,1:3)';
    for i=1:6
        arm_pos = copter_pos + C_bI' * [qp.x(i); qp.y(i); 0];
        orientation_plot_arms{i}.XData = [copter_pos(1) arm_pos(1)];
        orientation_plot_arms{i}.YData = [copter_pos(2) arm_pos(2)];
        orientation_plot_arms{i}.ZData = [copter_pos(3) arm_pos(3)];
        orientation_plot_armends{i}.XData = arm_pos(1);
        orientation_plot_armends{i}.YData = arm_pos(2);
        orientation_plot_armends{i}.ZData = arm_pos(3) + eps;
        orientation_plot_txtlabels{i}.Position = arm_pos + eps;
    end
    xlim(ax5, copter_pos(1) + [-qp.l qp.l]*2)
    ylim(ax5, copter_pos(2) + [-qp.l qp.l]*2)

    % --- ax6 ---
    axes_coords = C_bI' * (qp.l * eye(3));
    orientation_plot_xaxis.XData = [0 axes_coords(1,1)];
    orientation_plot_xaxis.YData = [0 axes_coords(2,1)];
    orientation_plot_xaxis.ZData = [0 axes_coords(3,1)];
    orientation_plot_yaxis.XData = [0 axes_coords(1,2)];
    orientation_plot_yaxis.YData = [0 axes_coords(2,2)];
    orientation_plot_yaxis.ZData = [0 axes_coords(3,2)];
    orientation_plot_zaxis.XData = [0 axes_coords(1,3)];
    orientation_plot_zaxis.YData = [0 axes_coords(2,3)];
    orientation_plot_zaxis.ZData = [0 axes_coords(3,3)];  
        


    drawnow
    title(ax1, sprintf("t=%.2f [s]", t_render(k) ))
end