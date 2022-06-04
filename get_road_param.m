function [road] = get_road_param()
    
    field_param = get_field_param();
    field_width = field_param.width;

    % параметры дороги
    road = struct();
    
    road.lane_width = 3.6; % ширина одной полосы (м)
    road.lane_cnt = 3; % количество полос движения (м)
    road.left_side_x = 0.5 * (field_width - ...
        road.lane_width * road.lane_cnt); % положение левой границы дороги (м)
    
    road.friction_coef = 0.75; % коэффициент трения резина-асфальт [0.7; 0.8]
    road.friction_ratio = 0.8; % отношение силы трения качения колёс к максимальной силе трения покоя
    road.turn_margin = 0.8; % запас для центробежной силы при повороте, чтобы не было заноса / сноса осей и т.п.

    assert_road_param(road);
    
end


function assert_road_param(road)

    field_param = get_field_param();
    field_width = field_param.width;
    
    lane_width = road.lane_width;
    lane_cnt = road.lane_cnt;
    left_side_x = road.left_side_x;
    friction_coef = road.friction_coef;
    friction_ratio = road.friction_ratio;
    turn_margin = road.turn_margin;
    
    assert(lane_width > 0, 'road.lane_width <= 0');
    assert(lane_cnt > 0, 'road.lane_cnt <= 0');
    assert(left_side_x >= 0, 'road.left_side_x < 0');
    assert((left_side_x + lane_cnt * lane_width) <= field_width, 'right_side_x > field_width');
    assert(friction_coef > 0, 'road.friction_coef <= 0');
    assert(friction_coef < 1, 'road.friction_coef >= 1');
    assert(friction_ratio > 0, 'road.friction_ratio <= 0');
    assert(friction_ratio <= 1, 'road.friction_ratio > 1');
    assert(turn_margin > 0, 'road.turn_margin <= 0');
    assert(turn_margin <= 1, 'road.turn_margin > 1');
    
end