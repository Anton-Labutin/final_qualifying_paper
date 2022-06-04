function [time_grid, x_y_vx_vy_phi_grid, road_accident_happened] = ...
    car_1_braking_maneuver(x_y_vx_vy_phi, car_2_time_grid, car_2_x_y_vx_vy_phi_grid)

    print_opts = get_print_opts();
    print_headers = print_opts.print_headers;
    print_intermediate_results = print_opts.print_intermediate_results;

    if print_headers
        disp('START 1st car braking');
    end
                    
% выполнение манёвра "торможение" 1м автомобилем после движения по реакции                                    
  
    road_accident_happened = false;

% ---- торможение

    [braking_time_grid, braking_x_y_vx_vy_phi_grid, braking_accident_happened] = ...
        braking_motion(x_y_vx_vy_phi, car_2_time_grid, car_2_x_y_vx_vy_phi_grid);
                    
    if print_intermediate_results
        print_car_motion_results( ...
            'Торможение 1-го автомобиля', ...
            braking_time_grid, ...
            braking_x_y_vx_vy_phi_grid ...
        );
    end
        
    if print_headers
        disp('FINISH 1st car braking');
    end
                    
    time_grid = braking_time_grid;
    x_y_vx_vy_phi_grid = braking_x_y_vx_vy_phi_grid;
                
    if ~braking_accident_happened
                       
        % не произошло столкновение при торможении 1-го автомобиля
                        
        braking_time_finish = braking_time_grid(end);
        left_after_braking_time_mask = (car_2_time_grid >= braking_time_finish);
        left_after_braking_time_grid = car_2_time_grid(left_after_braking_time_mask); 
                          
        if numel(left_after_braking_time_grid) > 1
        
            % время моделирования не истекло при торможении 1-го автомобиля
            
            % движение 1-го автомобиля по реакции с возможным отслеживанием
            % столкновения
            
            after_braking_x_y_vx_vy_phi = ...
                get_car_x_y_vx_vy_phi(braking_x_y_vx_vy_phi_grid(end, :));
            car_2_x_y_vx_vy_phi_grid_after_braking = ...
                car_2_x_y_vx_vy_phi_grid(left_after_braking_time_mask, :);
            
            [reaction_time_grid, reaction_x_y_vx_vy_phi_grid, reaction_accident_happened] = ...
                reaction_motion_after_braking(after_braking_x_y_vx_vy_phi, ...
                    left_after_braking_time_grid, ...
                    car_2_x_y_vx_vy_phi_grid_after_braking ...
                );
            
            if print_intermediate_results
                print_results( ...
                    'Движение 1-го автомобиля по реакции после торможения', ...
                    reaction_time_grid, ...
                    reaction_x_y_vx_vy_phi_grid...
                );
            end
 
            time_grid = [time_grid, reaction_time_grid];
            x_y_vx_vy_phi_grid = [x_y_vx_vy_phi_grid; reaction_x_y_vx_vy_phi_grid];
                
            if ~reaction_accident_happened
        
                % не произошло столкновение при движении 1-го автомобиля по реакции после торможения
        
                after_reaction_motion_time_mask = (car_2_time_grid >= reaction_time_grid(end));
                after_reaction_motion_time_grid = time_grid(after_reaction_motion_time_mask);

                if (numel(after_reaction_motion_time_grid) > 1)
        
                    % время моделирования не истекло при движении 1-го автомобиля
                    % по реакции после торможения
            
                    x_y_vx_vy_phi = get_car_x_y_vx_vy_phi(reaction_x_y_vx_vy_phi_grid(end, :));
                    
                    car_2_x_y_vx_vy_phi_grid_after_reaction = car_2_x_y_vx_vy_phi_grid(after_reaction_motion_time_mask, :);
                    car_2_x_y_vx_vy_phi = ...
                        get_car_x_y_vx_vy_phi(car_2_x_y_vx_vy_phi_grid_after_reaction(1, :));
            
                    v = norm([x_y_vx_vy_phi(3), x_y_vx_vy_phi(4)]);
                    car_2_v = norm([car_2_x_y_vx_vy_phi(3), car_2_x_y_vx_vy_phi(4)]);
            
                    if abs(car_2_v - v) > 1e-1
                    
                        [ speedup_time_grid, ...
                          speedup_x_y_vx_vy_phi_grid ...
                        ] = speedup_after_reaction( ...
                                x_y_vx_vy_phi, ...
                                after_reaction_motion_time_grid, ...
                                car_2_x_y_vx_vy_phi ...
                            );
                                        
                        if print_intermediate_results
                            print_results( ...
                                'Прямолинейное движение 1-го автомобиля с ускорением', ...
                                speedup_time_grid, ...
                                speedup_x_y_vx_vy_phi_grid...
                            );
                        end
                            
                        time_grid = [ ...
                            time_grid, ...
                            speedup_time_grid(2 : end)...
                        ];
                         
                        x_y_vx_vy_phi_grid = [ ...
                            x_y_vx_vy_phi_grid; ...
                            speedup_x_y_vx_vy_phi_grid(2 : end, :) ...
                        ];
                
                    end
                    
                else
                    
                    % время моделирования истекло при движении 1-го
                    % автомобиля по реакции после торможения
                    
                end
                            
            else
                
                % произошло стокновение при движении 1-го автомобиля по
                % реакции после торможения
                
                if print_headers
                    disp('Road accident happened');
                end
                
                road_accident_happened = true;
                
            end
          
        else
            
            % время моделирования истекло при торможении 1-го автомобиля
        
        end 
                            
    else
                        
        % произошло столкновение при торможении 1-го автомобиля  
                        
        if print_headers
            disp('Road accident happened');
        end
                        
        road_accident_happened = true;
                            
    end

end


function [time_grid, x_y_vx_vy_phi_grid, road_accident_happened] = ...
    braking_motion(x_y_vx_vy_phi, car_2_time_grid, car_2_x_y_vx_vy_phi_grid)

    v = norm([x_y_vx_vy_phi(3), x_y_vx_vy_phi(4)]);
    braking_acc = get_car_1_braking_motion_acc();
    
    braking_motion_param = struct(field_names.x, x_y_vx_vy_phi(1), ...
                                  field_names.y, x_y_vx_vy_phi(2), ...
                                  field_names.v, v, ...
                                  field_names.phi, x_y_vx_vy_phi(5), ...
                                  field_names.acc_tan, braking_acc, ...
                                  field_names.t, car_2_time_grid);
                    
    opts = get_start_ode_opts();
    opts.collision_braking_opts = struct( ...
        'car_2_x_y_vx_vy_phi_grid', car_2_x_y_vx_vy_phi_grid, ...
        field_names.t, car_2_time_grid, ...
        field_names.acc_tan, braking_acc ...
    );
                
    braking_sol = move_forward(braking_motion_param, opts);
                    
    time_grid = braking_sol.x;
    x_y_vx_vy_phi_grid = (braking_sol.y)';
    
    road_accident_happened = false;
    if isfield(braking_sol, 'xe') && (numel(braking_sol.xe) > 0) && (braking_sol.ie(1) == 1)
        road_accident_happened = true;
    end

end


function [time_grid, x_y_vx_vy_phi_grid, road_accident_happened] = ...
    reaction_motion_after_braking(...
        x_y_vx_vy_phi, car_2_time_grid, car_2_x_y_vx_vy_phi_grid )

    car_param = get_car_param();
    car_reaction_time = car_param.time_reaction;
    
    reaction_motion_time_mask = (car_2_time_grid <= (car_2_time_grid(1) + car_reaction_time));
    reaction_motion_time_grid = car_2_time_grid(reaction_motion_time_mask);
        
    car_2_reaction_x_y_vx_vy_phi_grid = ...
        car_2_x_y_vx_vy_phi_grid(reaction_motion_time_mask, :);
    
    v = norm([x_y_vx_vy_phi(3), x_y_vx_vy_phi(4)]);
    
    reaction_motion_param = struct(...
            'x', x_y_vx_vy_phi(1), ...
            'y', x_y_vx_vy_phi(2), ...
            'v', v, ...
            'phi', x_y_vx_vy_phi(5), ...
            'acc_tan', 0, ...
            't', reaction_motion_time_grid ...
    );
    
    opts.ode_opts = get_start_ode_opts();
    opts.intersection_opts = struct( ...
        'x', car_2_reaction_x_y_vx_vy_phi_grid(:, 1), ...
        'y', car_2_reaction_x_y_vx_vy_phi_grid(:, 2), ...
        'phi', car_2_reaction_x_y_vx_vy_phi_grid(:, 5), ...
        't', reaction_motion_time_grid ...
    );

    reaction_motion_sol = move_forward(reaction_motion_param, opts);
    time_grid = reaction_motion_sol.x;
    x_y_vx_vy_phi_grid = (reaction_motion_sol.y)';
    
    road_accident_happened = false;
    
    if isfield(reaction_motion_sol, 'xe') && (numel(reaction_motion_sol.xe) > 0)
        road_accident_happened = true;
    end

end


function [time_grid, x_y_vx_vy_phi_grid] = ...
    speedup_after_reaction(x_y_vx_vy_phi, car_2_time_grid, car_2_x_y_vx_vy_phi)

    [x, y, vx, vy, phi] = get_car_x_y_vx_vy_phi(x_y_vx_vy_phi);
    
    [car_2_x, car_2_y, car_2_vx, car_2_vy, car_2_phi] = ...
        get_car_x_y_vx_vy_phi(car_2_x_y_vx_vy_phi);
            
    v = norm([vx, vy]);
    car_2_v = norm([car_2_vx, car_2_vy]);
            
    car_param = get_car_param();
    safe_dist_ahead = car_param.safe_dist_ahead;
                
    distance = (car_2_y - (y + car_length)) + safe_dist_ahead;
    acc_tan = (car_2_v^2 - v^2) / (2 * distance);
        
    speedup_param = struct(...
        'x', x, ...
        'y', y, ...
        'v', v, ...
        'phi', phi, ...
        'acc_tan', acc_tan, ...
        't', car_2_time_grid ...
    );
    
    opts.speed_opts = struct('v_finish', car_2_v, 'eps', 1e-2);
    
    opts.ode_opts = get_start_ode_opts();
    opts.ode_opts = odeset(opts.ode_opts, 'RelTol', 1e-3, 'AbsTol', 1e-3);
        
    speedup_sol = move_forward(speedup_param, opts);
    speedup_time_grid = speedup_sol.x;
    speedup_x_y_vx_vy_phi_grid = (speedup_sol.y)';
        
    time_grid = [time_grid, speedup_time_grid(2 : end)];
            
    x_y_vx_vy_phi_grid = [ ...
        x_y_vx_vy_phi_grid; ...
        speedup_x_y_vx_vy_phi_grid(2 : end, :) ...
    ];
    
end