function [ode_opts] = get_ode_opts(opts, time_start)

    event_names = get_event_names();
    field_names = get_field_names();
    
    print_opts = get_print_opts();
    print_headers = print_opts.print_headers;

    ode_opts = opts.ode_opts;
    
    if isfield(opts, event_names.time_limit_opts)
        time_limit_opts = getfield(opts, event_names.time_limit_opts);
        time_finish = getfield(time_limit_opts, field_names.t);
        
        time_limit_opts = setfield(time_limit_opts, field_names.t, time_finish - time_start);
        
        if isfield(opts, event_names.intersection_opts)
            
            if print_headers
                disp('Time_limit_intersection events');
            end
            
            intersection_opts = getfield(opts, event_names.intersection_opts);
            car_2_time_grid = getfield(intersection_opts, field_names.t);
            intersection_opts = setfield(intersection_opts, field_names.t, car_2_time_grid - time_start);
            
            ode_opts = odeset(ode_opts, 'Events', ...
                @(t, x_y_vx_vy_phi) time_limit_intersection_events_fun(t, ...
                    x_y_vx_vy_phi, ...
                    time_limit_opts, ...
                    intersection_opts));
        else
            if print_headers
                disp('Time_limit events');
            end
            
            ode_opts = odeset(ode_opts, 'Events', ...
            @(t, x_y_vx_vy_phi) time_limit_events_fun(t, ...
                x_y_vx_vy_phi, ...
                time_limit_opts));
        end
    else
        if isfield(opts, event_names.intersection_opts)
            intersection_opts = getfield(opts, event_names.intersection_opts);
            car_2_time_grid = getfield(intersection_opts, field_names.t);
            intersection_opts = setfield(intersection_opts, field_names.t, car_2_time_grid - time_start);
        
            if isfield(opts, event_names.motion_opts)
                
                if print_headers
                    disp('Intersection_motion events');
                end
                
                motion_opts = getfield(opts, event_names.motion_opts);
            
                ode_opts = odeset(ode_opts, 'Events', ...
                    @(t, x_y_vx_vy_phi) intersection_motion_events_fun(t, ...
                        x_y_vx_vy_phi, ...
                        intersection_opts, ...
                        motion_opts));
            else
                
                if print_headers
                    disp('Intersection events');
                end
                
                ode_opts = odeset(ode_opts, 'Events', ...
                    @(t, x_y_vx_vy_phi) intersection_events_fun(t, ...
                        x_y_vx_vy_phi, ...
                        intersection_opts));
            end
        else
            if isfield(opts, event_names.collision_speedup_opts)
                
                if print_headers
                    disp('Collision & speedup events');
                end
                
                collision_speedup_opts = getfield(opts, event_names.collision_speedup_opts);
                car_2_time_grid = getfield(collision_speedup_opts, field_names.t);
                collision_speedup_opts = setfield(collision_speedup_opts, field_names.t, car_2_time_grid - time_start);
                
                ode_opts = odeset(ode_opts, 'Events', ...
                    @(t, x_y_vx_vy_phi) ode45_events_collision_speedup_fun( ...
                        t, x_y_vx_vy_phi, collision_speedup_opts) ...
                );
            
            else
                if isfield(opts, 'collision_braking_opts')
                   
                    if print_headers
                        disp('Collision & braking events');
                    end
                    
                    collision_braking_opts = getfield(opts, 'collision_braking_opts');
                    car_2_time_grid = getfield(collision_braking_opts, field_names.t);
                    collision_braking_opts = setfield(collision_braking_opts, field_names.t, car_2_time_grid - time_start);

                    ode_opts = odeset(ode_opts, 'NonNegative', 4, 'Events', ...
                        @(t, x_y_vx_vy_phi) ode45_events_collision_braking_fun( ...
                            t, x_y_vx_vy_phi, collision_braking_opts) ...
                    );
                    
                else
                    
                    if isfield(opts, 'speed_opts')
                
                        if print_headers
                            disp('Speed events');
                        end
                        
                        speed_opts = opts.speed_opts;
                    
                        ode_opts = odeset(ode_opts, 'Events', ...
                            @(t, x_y_vx_vy_phi) ode45_events_speed_fun( ...
                                t, x_y_vx_vy_phi, speed_opts) ...
                        );
                
                    end
                    
                end
                
            end
        end
    end
end