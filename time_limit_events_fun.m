function [value, isterminal, direction] = time_limit_events_fun(t, x_y_vx_vy_phi, time_limit_opts)

    field_names = get_field_names();

    time_finish = getfield(time_limit_opts, field_names.t);
    
    value = zeros(1, 1);
    isterminal = zeros(1, 1);
    direction = zeros(1, 1);
   
    value(1) = ((time_finish - t) > 0);
    isterminal(1) = 1;                 % terminate integration
    direction(1) = 0;                  % all zeros
end