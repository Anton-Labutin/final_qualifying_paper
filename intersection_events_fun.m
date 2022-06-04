function [value, isterminal, direction] = intersection_events_fun(t, x_y_vx_vy_phi, intersection_opts)

    field_names = get_field_names();
        
    car_1_x = x_y_vx_vy_phi(1);
    car_1_y = x_y_vx_vy_phi(2);
    car_1_phi = x_y_vx_vy_phi(5);
    
    car_2_x_grid = getfield(intersection_opts, field_names.x);
    car_2_y_grid = getfield(intersection_opts, field_names.y);
    car_2_phi_grid = getfield(intersection_opts, field_names.phi);
    
    time_grid = getfield(intersection_opts, field_names.t);
    
    t_prev_idx = find(time_grid <= t, 1, 'last');
    t_next_idx = find(time_grid >= t, 1);
    
    car_2_x_prev = car_2_x_grid(t_prev_idx);
    car_2_y_prev = car_2_y_grid(t_prev_idx);
    car_2_phi_prev = car_2_phi_grid(t_prev_idx);
    
    if (t_prev_idx ~= t_next_idx)
        
        time_prev = time_grid(t_prev_idx);
        time_next = time_grid(t_next_idx);
        time_duration = time_next - time_prev;
        
        coef = (t - time_prev) / time_duration;
        
        car_2_x_next = car_2_x_grid(t_next_idx);
        car_2_y_next = car_2_y_grid(t_next_idx);
        car_2_phi_next = car_2_phi_grid(t_next_idx);
    
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
    
    value = [(~are_intersected(frame_1, frame_2))]; % t = t_end
    isterminal = [1];                               % terminate integration
    direction = [0];                                % all zeros
   
end