function [value, isterminal, direction] = ode45_events_collision_speedup_fun( ...
    t, x_y_vx_vy_phi, collision_opts)

    field_names = get_field_names();
    car_param = get_car_param();
    car_length = car_param.length;
    
    car_1_x = x_y_vx_vy_phi(1);
    car_1_y = x_y_vx_vy_phi(2);
    car_1_phi = x_y_vx_vy_phi(5);
    
    car_2_x_grid = getfield(collision_opts, field_names.x);
    car_2_y_grid = getfield(collision_opts, field_names.y);
    car_2_phi_grid = getfield(collision_opts, field_names.phi);
    
    car_2_time_grid = getfield(collision_opts, field_names.t);
    
    time_prev_idx = find(car_2_time_grid <= t, 1, 'last');
    time_next_idx = find(car_2_time_grid >= t, 1);
    
    car_2_x_prev = car_2_x_grid(time_prev_idx);
    car_2_y_prev = car_2_y_grid(time_prev_idx);
    car_2_phi_prev = car_2_phi_grid(time_prev_idx);
    
    if (time_prev_idx ~= time_next_idx)
        
        time_prev = car_2_time_grid(time_prev_idx);
        time_next = car_2_time_grid(time_next_idx);
        time_duration = time_next - time_prev;
        
        coef = (t - time_prev) / time_duration;
        
        car_2_x_next = car_2_x_grid(time_next_idx);
        car_2_y_next = car_2_y_grid(time_next_idx);
        car_2_phi_next = car_2_phi_grid(time_next_idx);
    
        car_2_x = car_2_x_prev + coef * (car_2_x_next - car_2_x_prev);
        car_2_y = car_2_y_prev + coef * (car_2_y_next - car_2_y_prev);
        car_2_phi = car_2_phi_prev + coef * (car_2_phi_next - car_2_phi_prev);
    else
        car_2_x = car_2_x_prev;
        car_2_y = car_2_y_prev;
        car_2_phi = car_2_phi_prev;
    end
    
    frame_1 = get_car_frame(car_1_x, car_1_y, car_1_phi);
    frame_2 = get_car_frame(car_2_x, car_2_y, car_2_phi);
    
    car_2_upper_right_corner_y = frame_2(2, 3);
    
    safe_dist = get_safe_distance();
   
    value = zeros(1, 2);
    isterminal = zeros(1, 2);
    direction = zeros(1, 2);
    
    value(1) = ~are_intersected(frame_1, frame_2); % столкнулись ли машины
    isterminal(1) = 1;                             % terminate integration
    direction(1) = 0;                              % all zeros
    
    value(2) = ((car_1_y - car_2_upper_right_corner_y) <= ... 
                (car_length + safe_dist));
    isterminal(2) = 1;             
    direction(2) = 0;               

end