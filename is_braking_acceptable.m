function [acceptable] = is_braking_acceptable(...
    car_1_x_y_vx_vy_phi, car_2_x_y_vx_vy_phi)
    
    car_1_vx = car_1_x_y_vx_vy_phi(3);
    car_1_vy = car_1_x_y_vx_vy_phi(4);
    car_1_v = norm([car_1_vx, car_1_vy]);
    car_1_y = car_1_x_y_vx_vy_phi(2);

    car_2_x = car_2_x_y_vx_vy_phi(1);
    car_2_y = car_2_x_y_vx_vy_phi(2);
    car_2_phi = car_2_x_y_vx_vy_phi(5);
                
    car_2_frame = get_car_frame(car_2_x, car_2_y, car_2_phi);
            
    car_2_upper_left_corner_y = car_2_frame(2, 2);
    car_2_down_left_corner_y = car_2_frame(2, 1);
    
    car_1_braking_acc = get_car_1_braking_motion_acc();             
    car_1_max_braking_path = calc_braking_path(car_1_v, 0, -car_1_braking_acc);
                
    car_param = get_car_param();
    car_length = car_param.car_length;
    safe_dist_ahead = car_param.safe_dist_ahead;
    
    car_1_front_bumper_y = car_1_y + car_length;
    distance_y = car_2_down_left_corner_y - car_1_front_bumper_y;
    
    acceptable = false;
    if (((car_1_front_bumper_y < car_2_upper_left_corner_y) && ( distance_y <= 0)) ...
        || ((distance_y > 0) && (distance_y <= (car_1_max_braking_path + safe_dist_ahead)))) 
        acceptable = true;
    end

end