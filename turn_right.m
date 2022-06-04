function sol = turn_right(circle_motion_param, opts)

    print_opts = get_print_opts();
    print_headers = print_opts.print_headers;

    if print_headers
        disp('START turn_right');
    end

    [x_0, y_0, v_0, phi_0, acc_rot, R, t_grid] = ...
        get_circle_motion_param(circle_motion_param);
    
    t_start = t_grid(1);
    t_grid = t_grid - t_start;
    
    ode_opts = get_ode_opts(opts, t_start);
    
    vx_0 = v_0 * sin(phi_0);
    vy_0 = v_0 * cos(phi_0);
    
    omega_0 = v_0 / R;
    
    sol = ode45( ...
        @(t, xy) turn_right_motion_eq(t, xy, R, acc_rot), ...
        t_grid, ...
        [x_0, y_0, vx_0, vy_0, phi_0, omega_0], ...
        ode_opts ...
    );
            
    if (numel(t_grid) > 2)
        t_grid = t_grid(t_grid <= sol.x(end));
        sol.y = deval(sol, t_grid);
        sol.x = t_grid;
        
        if isfield(sol, 'xe') && (numel(sol.xe) > 0)
            sol.xe = sol.x(end);
        end
    end
    
    sol.y(end, :) = [];
    
    sol.x = sol.x + t_start;
    
    if isfield(sol, 'xe') && (numel(sol.xe) > 0)
        sol.xe = sol.xe + t_start;
    end
    
    if print_headers
        disp('FINISH turn_right');
    end
    
end