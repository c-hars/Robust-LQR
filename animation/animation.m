%% Ball-and-stick hexacopter animation

Ts_render = 1/60;
t_render = 0:Ts_render:10;

x_I = interp1(t, X(:,1), t_render);
y_I = interp1(t, X(:,2), t_render);
z_I = interp1(t, X(:,3), t_render);

phi   = interp1(t, X(:,7), t_render);
theta = interp1(t, X(:,8), t_render);
psi   = interp1(t, X(:,9), t_render);


%%


% --- Parameters ---
base_color = [0 0 1]; % blue hexacopter arms
% filename = 'animation.gif';
% v = VideoWriter('animation.mp4', 'MPEG-4');
% v.Quality = 100;
% v.FrameRate = 60;
% open(v);

% --- Figure setup ---
figure(1); clf; hold on; grid on
xlabel('x'); ylabel('y'); zlabel('z')
view(225,30); axis equal

% --- Add origin marker ---
origin = plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',12);

% --- Initialize arms ---
arms = gobjects(1,6);
for i = 1:6
    arms(i) = plot3([0 qp.l*cos(qp.phi(i))], [0 qp.l*sin(qp.phi(i))], [0 0], ...
        'LineWidth', 4, 'Color', [base_color 0.4]);
end

% --- Initialize CoM ---
body = plot3(0,0,0,'o','MarkerFaceColor',base_color,'MarkerSize',12);

% --- Animation loop ---
for k = 1:length(t_render)

    C_bI = C_x(phi(k))*C_y(theta(k))*C_z(psi(k));
    C_bI = C_bI';

    t_k = t_render(k);
    available_thrust = thrust_fcn(t_k);
    arm_alphas = 0.1 + 0.9*(available_thrust); % fade transparency when motor is partially/fully disabled

    for i = 1:6
        arm_end = C_bI * [qp.x(i); qp.y(i); 0];
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
    
    % xlim([min([x_c y_c]), max([x_c y_c])] + [-1 1]*qp.l)
    % ylim([min([x_c y_c]), max([x_c y_c])] + [-1 1]*qp.l)
    % zlim([min(z_c), max(z_c)] + [-1 1]*qp.l)
    xlim([-0.2000    1.7285])
    ylim([-0.2000    1.7285])
    zlim([-0.3954    0.2000])
    
    drawnow
    title(sprintf("%.0f%% thrust available from Propeller 1, t=%.2f [s]", 100*(min(available_thrust)), t_render(k) ))
    % pause(0.05)

    % % Capture the figure as an image
    % frame = getframe(gcf);
    % im = frame2im(frame);
    % [A,map] = rgb2ind(im,256);
    
    % % Write to GIF and mp4
    % if k == 1
    %     imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',Ts_render);
    % else
    %     imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',Ts_render);
    % end
    % writeVideo(v,im);
end
% close(v)