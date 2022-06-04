function [total_time_grid, x_y_vx_vy_phi_grid, road_accident_happened] = straight_motion_after_braking(...
    time_grid, ...
    motion_param, ...
    car_2_x_y_vx_vy_phi_grid ...
)

% Прямолинейное движение 1-го авомобиля после торможения в ответ на
% перестроение 2-го автомобиля

    total_time_grid = [];
    x_y_vx_vy_phi_grid = [];
    road_accident_happened = false;

    car_param = get_car_param();
    car_length = car_param.length;
    car_reaction_time = car_param.time_reaction;
    
    opts.ode_opts = get_start_ode_opts();
    
% Движение после торможния по реакции с возможным отслеживанием
% столкновения
        
    reaction_motion_time_mask = (time_grid <= (time_grid(1) + car_reaction_time));
    reaction_motion_time_grid = time_grid(reaction_motion_time_mask);
        
    reaction_motion_car_2_x_y_vx_vy_phi_grid = ...
        car_2_x_y_vx_vy_phi_grid(reaction_motion_time_mask, :);
    
    reaction_motion_param = struct(...
            'x', motion_param.x, ...
            'y', motion_param.y, ...
            'v', motion_param.v, ...
            'phi', motion_param.phi, ...
            'acc_tan', 0, ...
            't', reaction_motion_time_grid ...
    );
    
    opts.intersection_opts = struct( ...
        'x', reaction_motion_car_2_x_y_vx_vy_phi_grid(:, 1), ...
        'y', reaction_motion_car_2_x_y_vx_vy_phi_grid(:, 2), ...
        'phi', reaction_motion_car_2_x_y_vx_vy_phi_grid(:, 5), ...
        't', reaction_motion_time_grid ...
    );

    reaction_motion_sol = move_forward(reaction_motion_param, opts);
    reaction_motion_time_grid = reaction_motion_sol.x;
    reaction_motion_x_y_vx_vy_phi_grid = (reaction_motion_sol.y)';
    
    opts = rmfield(opts, 'intersection_opts');
    
    total_time_grid = reaction_motion_time_grid;
    x_y_vx_vy_phi_grid = reaction_motion_x_y_vx_vy_phi_grid;
    
    if ((~isfield(reaction_motion_sol, 'xe')) || (numel(reaction_motion_sol.xe) == 0))
        
        % не произошло столкновение при движении 1-го авто по реакции после
        % торможения
        
        after_reaction_motion_time_mask = (time_grid >= reaction_motion_time_grid(end));
        after_reaction_motion_time_grid = time_grid(after_reaction_motion_time_mask);

        if (numel(after_reaction_motion_time_grid) > 1)
        
            % время моделирования не истекло при движении 1-го автомобиля
            % по реакции после торможения в ответ на перестроение 2-го
            % автомобиля
            
            after_reaction_motion_car_2_x_y_vx_vy_phi_grid = ...
                car_2_x_y_vx_vy_phi_grid(after_reaction_motion_time_mask, :);
            
            [car_2_x, car_2_y, car_2_vx, car_2_vy, car_2_phi] = ...
                get_car_x_y_vx_vy_phi(after_reaction_motion_car_2_x_y_vx_vy_phi_grid(1, :));
    
            [x, y, vx, vy, phi] = ...
                get_car_x_y_vx_vy_phi(reaction_motion_x_y_vx_vy_phi_grid(end, :));
            
            v = norm([vx, vy]);
            car_2_v = norm([car_2_vx, car_2_vy]);
            
            acceleration_needed = false;
            
            if abs(v - car_2_v) > 1e-1
                
                acceleration_needed = true;
                safe_distance = get_safe_distance();
                
                distance = (car_2_y - (y + car_length)) + safe_distance;
                acc_tan = (car_2_v^2 - v^2) / (2 * distance);
        
                motion_with_acc_param = struct(...
                    'x', x, ...
                    'y', y, ...
                    'v', v, ...
                    'phi', phi, ...
                    'acc_tan', acc_tan, ...
                    't', after_reaction_motion_time_grid ...
                );
    
                opts.speed_opts = struct('v_finish', car_2_v, 'eps', 1e-2);
                opts.ode_opts = get_start_ode_opts();
                opts.ode_opts = odeset(opts.ode_opts, 'RelTol', 1e-3, 'AbsTol', 1e-3);
        
                motion_with_acc_sol = move_forward(motion_with_acc_param, opts);
                motion_with_acc_time_grid = motion_with_acc_sol.x;
                motion_with_acc_x_y_vx_vy_phi_grid = (motion_with_acc_sol.y)';
        
                total_time_grid = [ ...
                    total_time_grid, ...
                    motion_with_acc_time_grid(2 : end) ...
                ];
            
                x_y_vx_vy_phi_grid = [ ...
                    x_y_vx_vy_phi_grid; ...
                    motion_with_acc_x_y_vx_vy_phi_grid(2 : end, :) ...
                ];
        
                opts = rmfield(opts, 'speed_opts');
        
            else
                v = car_2_v;
            end
            
            if acceleration_needed
            
                [x, y, vx, vy, phi] = ...
                    get_car_x_y_vx_vy_phi(motion_with_acc_x_y_vx_vy_phi_grid(end, :));
        
                v = norm([vx, vy]);
        
                tmp = (after_reaction_motion_time_grid >= motion_with_acc_time_grid(end));
                after_reaction_motion_time_grid = after_reaction_motion_time_grid(tmp);
            
            end
            
            if ~acceleration_needed || (numel(after_reaction_motion_time_grid) > 1)
            
                % время моделирования не закончилось после возможного
                % прямолинейного движения 1-го автомобиля с ускорением
                % после его движения по реакции

                constant_motion_param = struct(...
                    'x', x, ...
                    'y', y, ...
                    'v', v, ...
                    'phi', phi, ...
                    'acc_tan', 0, ...
                    't', after_reaction_motion_time_grid ...
                );

                constant_motion_sol = move_forward(constant_motion_param, opts);
                constant_motion_time_grid = constant_motion_sol.x;
                constant_motion_x_y_vx_vy_phi_grid = (constant_motion_sol.y)';
    
                total_time_grid = [ ...
                    total_time_grid, ...
                    constant_motion_time_grid(2 : end) ...
                ];
            
                x_y_vx_vy_phi_grid = [ ...
                    x_y_vx_vy_phi_grid; ...
                    constant_motion_x_y_vx_vy_phi_grid(2 : end, :) ...
                ];
            
            end
    
        end
        
    else
        road_accident_happened = true;
    end
    
end