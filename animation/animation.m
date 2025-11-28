%% Ball-and-stick hexacopter animation

Ts_render = 0.05;
t_render = 0:Ts_render:10;

x_c = interp1(t, X(:,1), t_render);
y_c = interp1(t, X(:,2), t_render);
z_c = interp1(t, X(:,3), t_render);

phi   = interp1(t, X(:,7), t_render);
theta = interp1(t, X(:,8), t_render);
psi   = interp1(t, X(:,9), t_render);


%%


% --- Parameters ---
base_color = [0 0 1]; % blue hexacopter arms
filename = 'animation.gif';

% --- Figure setup ---
figure(1); clf; hold on; grid on
xlabel('x'); ylabel('y'); zlabel('z')
view(225,30); axis equal

% --- Add origin marker ---
origin = plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',4);

% --- Initialize arms ---
arms = gobjects(1,6);
for i = 1:6
    arms(i) = plot3([0 qp.l*cos(qp.phi(i))], [0 qp.l*sin(qp.phi(i))], [0 0], ...
        'LineWidth', 4, 'Color', [base_color 0.4]);
end

% --- Initialize CoM ---
body = plot3(0,0,0,'ko','MarkerFaceColor','b','MarkerSize',12);

% --- Animation loop ---
for k = 1:length(t_render)

    R = C_x(phi(k))*C_y(theta(k))*C_z(psi(k));
    R = R';

    t_k = t_render(k);
    available_thrust = thrust_fcn(t_k);
    arm_alphas = 0.1 + 0.9*(available_thrust); % fade transparency when motor is partially/fully disabled

    for i = 1:6
        arm_end = R * [qp.x(i); qp.y(i); 0];
        set(body, ...
            'XData', [x_c(k) x_c(k)], ...
            'YData', [y_c(k) y_c(k)], ...
            'ZData', [z_c(k) z_c(k)]);
        set(arms(i), ...
            'XData', [x_c(k) x_c(k)] + [0 arm_end(1)], ...
            'YData', [y_c(k) y_c(k)] + [0 arm_end(2)], ...
            'ZData', [z_c(k) z_c(k)] + [0 arm_end(3)]);
        set(arms(i),'Color',[base_color arm_alphas(i)]); % red & semi-transparent
    end
    
    xlim([min([x_c y_c]), max([x_c y_c])] + [-1 1]*qp.l)
    ylim([min([x_c y_c]), max([x_c y_c])] + [-1 1]*qp.l)
    zlim([min(z_c), max(z_c)] + [-1 1]*qp.l)
    
    drawnow
    title(sprintf("%.0f%% thrust available from Propeller 1, t=%.2f [s]", 100*(min(available_thrust)), t_render(k) ))
    % pause(0.05)

    % Capture the figure as an image
    frame = getframe(gcf);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);
    
    % Write to GIF
    if k == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',Ts_render);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',Ts_render);
    end    
end
