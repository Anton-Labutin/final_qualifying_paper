function [max_curvature] = calc_max_curvature(car_v)

    arguments
        car_v (1,1) double {mustBePositive}
    end

    road_param = get_road_param();

    friction_coef = road_param.friction_coef;
    friction_ratio = road_param.friction_ratio;
    turn_margin = road_param.turn_margin;

    g = get_free_fall_acc();
    
    max_curvature = friction_coef * friction_ratio * turn_margin * g / car_v^2;
    
end