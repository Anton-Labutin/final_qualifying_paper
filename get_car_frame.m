function frame_rotated = get_car_frame(x_center, y_center, rotation_angle)

    % (x_center, y_center) -  координаты центра задней оси автомобиля
    % rotation_angle - угол поворота автомобиля от оси OY (> 0 - по часовой
    % стрелке)
    
    car_param = get_car_param();
    car_length = car_param.length;
    car_width = car_param.width;
    
    % [левый нижний, левый верхний, правый верхний, правый нижний]
    frame_x_not_rotated = [x_center - car_width / 2, ...
                           x_center - car_width / 2, ...
                           x_center + car_width / 2, ...
                           x_center + car_width / 2];
                       
    frame_y_not_rotated = [y_center, ...
                           y_center + car_length, ...
                           y_center + car_length, ...
                           y_center];
    
    poly = polyshape(frame_x_not_rotated, frame_y_not_rotated);
    poly_rotated = rotate(poly, -rotation_angle * 180 / pi, [x_center, y_center]);
    frame_rotated = (poly_rotated.Vertices)';
    
end