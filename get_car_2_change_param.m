function [turn_1_param, turn_2_param] = get_car_2_change_param(v)

    arguments
        v (1, 1) double {mustBePositive}
    end
    
    road_param = get_road_param();
    lane_width = road_param.lane_width;
    
    max_curvature = calc_max_curvature(v);
    turn_R_min = 1 / max_curvature;
    
    turn_1_abs_mov_x = 0.5 * lane_width;
    assert(turn_1_abs_mov_x > 0, 'turn_1_abs_mov_x <= 0');
    
    curvature = max_curvature / 3;
    assert(curvature > 0, 'curvature <= 0');
    assert(curvature <= max_curvature, 'curvature > max_curvature');
    
    tmp = 1 - turn_1_abs_mov_x * curvature;
    assert(tmp >= -1, 'tmp < -1');
    assert(tmp <= 1, 'tmp > 1');
    rot_angle = acos(tmp);
    
    turn_1_R_wanted = 1 / curvature;
    turn_2_R_wanted = (lane_width - turn_1_abs_mov_x) / (1 - cos(rot_angle));
    assert(turn_2_R_wanted > 0, 'turn_2_R_wanted <= 0');
    
    turn_1_param = struct();
    turn_1_param.acc_rot = 0;
    turn_1_param.R = max(turn_1_R_wanted, turn_R_min);
    turn_1_param.time = rot_angle * turn_1_param.R / v;
    assert(turn_1_param.time > 0, 'turn_1_param.time <= 0');

    turn_2_param = struct();
    turn_2_param.acc_rot = 0;           
    turn_2_param.R = max(turn_2_R_wanted, turn_R_min);
    turn_2_param.time = turn_2_param.R * rot_angle / v;
    assert(turn_2_param.time > 0, 'turn_2_param.time <= 0');
    
end