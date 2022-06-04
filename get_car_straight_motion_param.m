function [x, y, v, phi, acc_tan, time_grid] = get_car_straight_motion_param(motion_param)
    
    field_names = get_field_names();

    x = getfield(motion_param, field_names.x);
    y = getfield(motion_param, field_names.y);
    v = getfield(motion_param, field_names.v);
    phi = getfield(motion_param, field_names.phi);
    acc_tan = getfield(motion_param, field_names.acc_tan);
    time_grid = getfield(motion_param, field_names.t);
end