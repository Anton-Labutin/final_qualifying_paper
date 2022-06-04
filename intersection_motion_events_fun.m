function [value, isterminal, direction] = intersection_motion_events_fun(t, x_y_vx_vy_phi, intersection_opts, motion_opts)

    field_names = get_field_names();
        
    car_1_x = x_y_vx_vy_phi(1);
    car_1_y = x_y_vx_vy_phi(2);
    car_1_phi = x_y_vx_vy_phi(5);
    
    car_2_x_grid = getfield(intersection_opts, field_names.x);
    car_2_y_grid = getfield(intersection_opts, field_names.y);
    car_2_phi_grid = getfield(intersection_opts, field_names.phi);
    
    time_grid = getfield(intersection_opts, field_names.t);
    
    time_prev_idx = find(time_grid <= t, 1, 'last');
    time_next_idx = find(time_grid >= t, 1);
    
    car_2_x = 0.5 * (car_2_x_grid(time_prev_idx) + car_2_x_grid(time_next_idx));
    car_2_y = 0.5 * (car_2_y_grid(time_prev_idx) + car_2_y_grid(time_next_idx));
    car_2_phi = 0.5 * (car_2_phi_grid(time_prev_idx) + car_2_phi_grid(time_next_idx));
    
    frame_1 = get_car_frame(car_1_x, car_1_y, car_1_phi);
    frame_2 = get_car_frame(car_2_x, car_2_y, car_2_phi);
    
    car_1_vy = x_y_vx_vy_phi(4);
    car_1_v_finish = getfield(motion_opts, field_names.v);
    eps = get_eps();

   
    value = zeros(1, 2);
    isterminal = zeros(1, 2);
    direction = zeros(1, 2);
    
    value(1) = ~are_intersected(frame_1, frame_2); % столкнулись ли машины
    isterminal(1) = 1;                             % terminate integration
    direction(1) = 0;                              % all zeros
    
    % value(2) = (abs(v_end^2 - vx^2 - vy^2) > eps); % равна ли скорость машины v_end
    value(2) = ((car_1_vy - car_1_v_finish) > eps);
    isterminal(2) = 1;              % прекратить движение, когда скорость = v_end
    direction(2) = 0;               % all zeros

end