function [x, y, v, phi, acc_rot, R, t_grid] = get_circle_motion_param(circle_motion_param)

    field_names = get_field_names();

    x = getfield(circle_motion_param, field_names.x);
    y = getfield(circle_motion_param, field_names.y);
    v = getfield(circle_motion_param, field_names.v);
    phi = getfield(circle_motion_param, field_names.phi);
    acc_rot = getfield(circle_motion_param, field_names.acc_rot);
    R = getfield(circle_motion_param, field_names.R);
    t_grid = getfield(circle_motion_param, field_names.t);
end