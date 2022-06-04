function [car_1_motion_param, car_2_motion_param] = get_cars_start_motion_param()
    
    field_param = get_field_param();
    field_height = field_param.height;
    
    road_param = get_road_param();
    road_left_side_x = road_param.left_side_x;
    lane_width = road_param.lane_width;

    car_1_motion_param = struct();
    car_1_motion_param.x = road_left_side_x + 1.5 * lane_width;
    car_1_motion_param.y = field_height / 20;
    % car_1_motion_param.v_grid = (10 : 1 : 20);
    car_1_motion_param.v_grid = [11];
    car_1_motion_param.acc_tan = 0;

    car_2_motion_param = struct();
    car_2_motion_param.x = car_1_motion_param.x + lane_width;
    % car_2_motion_param.y_grid = car_1_motion_param.y + (0 : 1 : 20); 
    car_2_motion_param.y_grid = [car_1_motion_param.y + 8];
    car_2_motion_param.v = 10;
    car_2_motion_param.acc_tan = 0;
    
    assert_cars_motion_param(car_1_motion_param,car_2_motion_param);

end


function assert_cars_motion_param(car_1, car_2)

    field_param = get_field_param();
    field_height = field_param.height;
    
    road_param = get_road_param();
    road_left_side_x = road_param.left_side_x;
    lane_width = road_param.lane_width;
    lane_cnt = road_param.lane_cnt;
    
    car_1_x = car_1.x;
    car_1_y = car_1.y;
    
    car_2_x = car_2.x;
    car_2_y_grid = car_2.y_grid;

    assert((car_1_x > road_left_side_x) && ...
            (car_1_x < (road_left_side_x + lane_width * lane_cnt)), ...
            'car_1 is outside the road'); 
       
    assert(car_1_y > 0, 'car_1_y <= 0');
    assert(car_1_y < field_height, 'car_1_y >= field.height');
    
    assert(isfield(car_1, 'acc_tan'), 'car_1_motion_param.acc_tan not exists');
    
    assert((car_2_x > road_left_side_x) && ...
            (car_2_x < (road_left_side_x + lane_width * lane_cnt)), ...
            'car_2 is outside the road');
        
    assert(all(car_2_y_grid > 0), 'car_2_motion_param.v <= 0');
    
    assert(isfield(car_2, 'acc_tan'), 'car_2_motion_param.acc_tan not exists');
    
    assert(car_1_x < car_2_x, 'car_2 is not to the right from car 1');
end