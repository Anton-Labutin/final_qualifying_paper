function [value, isterminal, direction] = ode45_events_speed_fun( ...
    t, x_y_vx_vy_phi, speed_opts)
    
    vx = x_y_vx_vy_phi(3);
    vy = x_y_vx_vy_phi(4);
    v = norm([vx, vy]);
    
    v_finish = getfield(speed_opts, 'v_finish');
    eps = getfield(speed_opts, 'eps');
    
    value = zeros(1, 1);
    isterminal = zeros(1, 1);
    direction = zeros(1, 1);
    
    value(1) = (sqrt(v_finish^2 - v^2) < eps);
    isterminal(1) = 1;              
    direction(1) = 0;

end