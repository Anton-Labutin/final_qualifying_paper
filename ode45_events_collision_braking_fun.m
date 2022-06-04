function [value, isterminal, direction] = ode45_events_collision_braking_fun( ...
    t, x_y_vx_vy_phi, collision_braking_opts)

    value = zeros(1, 2);
    isterminal = ones(1, 2);
    direction = zeros(1, 2);

    field_names = get_field_names();
    
    car_2_time_grid = getfield(collision_braking_opts, field_names.t);
    car_2_x_y_vx_vy_phi_grid = getfield(collision_braking_opts, 'car_2_x_y_vx_vy_phi_grid');
    
    car_2_x_grid = car_2_x_y_vx_vy_phi_grid(:, 1);
    car_2_y_grid = car_2_x_y_vx_vy_phi_grid(:, 2);
    car_2_vx_grid = car_2_x_y_vx_vy_phi_grid(:, 3);
    car_2_vy_grid = car_2_x_y_vx_vy_phi_grid(:, 4);
    car_2_phi_grid = car_2_x_y_vx_vy_phi_grid(:, 5);
    
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
    
    frame_1 = get_car_frame(x_y_vx_vy_phi(1), x_y_vx_vy_phi(2), x_y_vx_vy_phi(5));
    frame_2 = get_car_frame(car_2_x, car_2_y, car_2_phi);
    
    intersection_result = are_intersected(frame_1, frame_2);
                  
    if ~intersection_result
        
        value(1) = 1;
        
        acc = collision_braking_opts.acc_tan;
        
        car_param = get_car_param();
        car_length = car_param.length;
        
        v = norm([x_y_vx_vy_phi(3), x_y_vx_vy_phi(4)]);
        max_braking_path = v^2 / (2 * -acc);
        safe_distance = get_safe_distance();
        
        car_2_down_left_corner_y = frame_2(2, 1);
        bumper_y = x_y_vx_vy_phi(2) + car_length;
        
        car_2_nearest_point_y = car_2_down_left_corner_y;
        
        distance_y = car_2_nearest_point_y - bumper_y;
        
        if ((max_braking_path + safe_distance) >= distance_y) 
            value(2) = 1;
        else
            value(2) = 0;
        end
        
    else
        value(2) = 1;
    end
    
end