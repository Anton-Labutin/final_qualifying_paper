function [speedup_acc] = get_car_1_speedup_motion_acc()

    road_param = get_road_param();
    friction_coef = road_param.friction_coef;
    friction_ratio = road_param.friction_ratio;

    g = get_free_fall_acc(); % ускорение свободного падения    
    
    max_acc = friction_coef * friction_ratio * g;  % как его вычислить???

    speedup_acc = max_acc; 
    
    assert(speedup_acc > 0, 'car_1_speedup_motion_acc <= 0');
end