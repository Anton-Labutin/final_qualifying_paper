function [acc] = get_car_1_braking_motion_acc()

    road_param = get_road_param();
    friction_coef = road_param.friction_coef;
    friction_ratio = road_param.friction_ratio;

    g = get_free_fall_acc(); % ускорение свободного падения    
    
    abs_acc = friction_coef * friction_ratio * g;
    acc = -abs_acc;
    
    assert(acc < 0, 'car_1_braking_motion: acc >= 0');

end