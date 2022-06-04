function [value, isterminal, direction] = expect_lane_change_fun(t, x_y_vx_vy_phi_omega, t_change, eps)
    value = [];
    value(1) = (abs(t_change - t) > eps); % t <= t_change
    
    isterminal = [1];                     % termintate integration
    direction = [0];                      % locate all zeros
end