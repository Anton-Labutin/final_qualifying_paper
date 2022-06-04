function [value, isterminal, direction] = time_limit_and_intersection_events_fun(t, x_y_vx_vy_phi, time_limit_opts, intersection_opts)

    field_names = get_field_names();

    t_end = getfield(time_limit_opts, field_names.t);
    % eps = get_field(time_limit_opts, field_names.eps);
        
    x_1 = x_y_vx_vy_phi(1);
    y_1 = x_y_vx_vy_phi(2);
    phi_1 = x_y_vx_vy_phi(5);
    
    x_grid_2 = getfield(intersection_opts, field_names.x);
    y_grid_2 = getfield(intersection_opts, field_names.y);
    phi_grid_2 = getfield(intersection_opts, field_names.phi);
       
    width = getfield(intersection_opts, field_names.width);
    length = getfield(intersection_opts, field_names.length);
    car_dimensions = struct('length', length, 'width', width);
    
    t_grid = getfield(intersection_opts, field_names.t);
        
    t_prev_idx = find(t_grid <= t, 1, 'last');
    t_next_idx = find(t_grid >= t, 1);
    
    x_2 = 0.5 * (x_grid_2(t_prev_idx) + x_grid_2(t_next_idx));
    y_2 = 0.5 * (y_grid_2(t_prev_idx) + y_grid_2(t_next_idx));
    phi_2 = 0.5 * (phi_grid_2(t_prev_idx) + phi_grid_2(t_next_idx));
    
    frame_1 = get_car_frame(x_1, y_1, phi_1, car_dimensions);
    frame_2 = get_car_frame(x_2, y_2, phi_2, car_dimensions);
    
%     disp('t_end = ');
%     disp(t_end);
%     disp('t = ');
%     disp(t);

    value = zeros(1, 2);
    isterminal = zeros(1, 2);
    direction = zeros(1, 2);
    
    % value(1) = (abs(t_end - t) > eps); % t = t_end
    value(1) = ((t_end - t) > 0);
    isterminal(1) = 1;                 % terminate integration
    direction(1) = 0;                  % all zeros
    
    value(2) = ~are_intersected(frame_1, frame_2); % столкнулись ли машины
    isterminal(2) = 1;                             % terminate integration
    direction(2) = 0;                              % all zeros
end