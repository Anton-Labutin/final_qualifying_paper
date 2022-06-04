function [turn_1_param, turn_2_param] = get_car_1_change_param(v, curvature)
    
    arguments
        v (1, 1) double {mustBePositive}
        curvature (1, 1) double {mustBePositive}
    end

    road_param = get_road_param();

    lane_width = road_param.lane_width;
    
    turn_1_abs_mov_x = 0.5 * lane_width;
    assert(turn_1_abs_mov_x > 0, 'turn_1_abs_mov_x <= 0');
    
    turn_1_param = struct();
    turn_1_param.acc_rot = 0;
    turn_1_param.R = 1 / curvature;
    
    tmp = 1 - turn_1_abs_mov_x * curvature;
    assert(tmp >= -1, 'tmp < -1');
    assert(tmp <= 1, 'tmp > 1');
    rot_angle = acos(tmp);

    turn_1_param.time = rot_angle * turn_1_param.R / v;
    assert(turn_1_param.time > 0, 'turn_1_param.time <= 0');
    
    turn_2_param = struct();
    turn_2_param.acc_rot = 0;
    % радиус поворота направо, при котором 1-й автомобиль
    % выедет на середину полосы параллельно обочине
    turn_2_param.R = (lane_width - turn_1_abs_mov_x) / (1 - cos(rot_angle));
    assert(turn_2_param.R > 0, 'turn_2_param.R <= 0');
    turn_2_param.time =  turn_2_param.R * rot_angle / v;
    assert(turn_2_param.time > 0, 'turn_2_param.time <= 0');
    
end