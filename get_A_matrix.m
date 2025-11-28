% function A = get_A_matrix()
% 
%     % Construct A matrix (continous time)
%     A = zeros(12);
% 
%     % Position kinematics
%     A(1,4) = 1;   % dx_c/dt = v_c_x
%     A(2,5) = 1;   % dy_c/dt = v_c_y
%     A(3,6) = 1;   % dz_c/dt = v_c_z
% 
%     % Translational acceleration
%     g = 9.81;
%     A(4,8) =  g;  % ẍ_c ≈  g * theta
%     A(5,7) = -g;  % ÿ_c ≈ -g * phi
% 
%     % Angular kinematics
%     A(7,10) = 1;  % dϕ/dt = ω_x
%     A(8,11) = 1;  % dθ/dt = ω_y
%     A(9,12) = 1;  % dψ/dt = ω_z
% 
% end