function sol = move_forward(motion_param, opts)

    print_opts = get_print_opts();
    print_headers = print_opts.print_headers;

    if print_headers
        disp('START move_forward');
    end

    [x_start, y_start, v_start, phi_start, acc_tan, time_grid] = ... 
        get_car_straight_motion_param(motion_param);
    
    time_start = time_grid(1);
    time_grid = time_grid - time_start;
    
    ode_opts = get_ode_opts(opts, time_start);
    
    vx_start = v_start * sin(phi_start);
    vy_start = v_start * cos(phi_start);
        
    sol = ode45(@(t, xy) straight_motion_eq(t, xy, v_start, acc_tan), ...
                time_grid, ...
                [x_start, y_start, vx_start, vy_start, phi_start], ...
                ode_opts);
    
    if (numel(time_grid) > 2)
        time_grid = time_grid(time_grid <= sol.x(end));
        sol.y = deval(sol, time_grid);
        sol.x = time_grid;
        
        if isfield(sol, 'xe') && (numel(sol.xe) > 0)
            sol.xe = sol.x(end);
        end
    end
    
    sol.x = sol.x + time_start;
    
    if isfield(sol, 'xe') && (numel(sol.xe) > 0)
        sol.xe = sol.xe + time_start;
    end
    
    if print_headers
        disp('FINISH move_forward');
    end
    
end