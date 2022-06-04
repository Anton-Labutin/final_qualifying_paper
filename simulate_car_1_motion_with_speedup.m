function [total_time_grid, ...
          total_x_y_vx_vy_phi_grid, ...
          speedup_needed, ...
          road_accident_happened] = ...
    simulate_car_1_motion_with_speedup(motion_start_param, ...
                                       car_2_time_grid, ...
                                       car_2_x_y_vx_vy_phi_grid ...
                                       )

% ---- моделирование движения 1-го автомобиля до конца времени моделирования
% ---- либо до столкновения

    print_opts = get_print_opts();
    print_intermediate_results = print_opts.print_intermediate_results;
    print_headers = print_opts.print_headers;
    
    if print_headers
        disp('START 1st car modelling: speedup while 2d car change');
    end
    
    motion_time_param = get_motion_time_param();
    car_2_change_time_start = motion_time_param.change;

    x_start = motion_start_param.x;
    y_start = motion_start_param.y;
    v_start = motion_start_param.v;
    acc_tan = motion_start_param.acc_tan;
    
    car_param = get_car_param();
    car_length = car_param.length;
    car_reaction_time = car_param.time_reaction;
    speedup_acc = car_param.max_speedup_acc;
    
    road_accident_happened = false;
    speedup_needed = false;
    
    field_names = get_field_names();
    event_names = get_event_names();
        
% -------- прямолинейное движение 1-го автомобиля до начала перестроения 2-го 
% -------- автомобиля    

    if print_headers
        disp('START 1st car straight motion untill 2d car lane change');
    end
    
    straight_motion_time_mask = (car_2_time_grid <= car_2_change_time_start);
    straight_motion_time_grid = car_2_time_grid(straight_motion_time_mask);

    straight_motion_param = ...
        get_car_straight_motion_param_struct(x_start, ...
                                             y_start, ...
                                             v_start, ...
                                             0, ...
                                             acc_tan, ...
                                             straight_motion_time_grid);
           
    opts.ode_opts = get_start_ode_opts();
        
    straight_motion_sol = move_forward(straight_motion_param, ...
                                       opts);
        
    straight_motion_time_grid = straight_motion_sol.x;
    straight_motion_x_y_vx_vy_phi_grid = (straight_motion_sol.y)'; 
      
    if print_intermediate_results
        print_car_motion_results('Прямолинейное движение 1-го автомобиля до начала перестроения 2-го', ...
                                 straight_motion_time_grid, ....
                                 straight_motion_x_y_vx_vy_phi_grid);
    end
    
    if print_headers
        disp('FINISH 1st car straight motion untill 2d car lane change');
    end
    
    total_time_grid = straight_motion_time_grid;
    total_x_y_vx_vy_phi_grid = straight_motion_x_y_vx_vy_phi_grid;
                     
    if (car_2_time_grid(~straight_motion_time_mask) > 0)
        
        % время моделирования не закончилось при прямолинейном движении
        % 1-го автомобиля до начала перестроения 2-го
        
        [x, y, vx, vy, phi] = ...
            get_car_x_y_vx_vy_phi(straight_motion_x_y_vx_vy_phi_grid(end, :));    
                    
% -------- движение 1-го автомобиля по реакции

        if print_headers
            disp('START 1st car reaction motion');
        end
        
        reaction_time_finish = car_2_change_time_start + car_reaction_time;
        
        reaction_motion_time_mask = (...
                (car_2_time_grid >= car_2_change_time_start) & ...
                (car_2_time_grid <= reaction_time_finish) ...
        );
                
        reaction_motion_time_grid = ...
            car_2_time_grid(reaction_motion_time_mask);
            
        v = norm([vx, vy]);

        reaction_motion_param = ...
            get_car_straight_motion_param_struct(x, y, ...
                                                 v, phi, ...
                                                 acc_tan, ...
                                                 reaction_motion_time_grid);
                                                 
        % параметры для отслеживания столкновения за время движения 
        % 1-го автомобиля по реации
            
        car_2_change_time_start_index = find(car_2_time_grid == car_2_change_time_start, 1);
        reaction_motion_time_grid_len = numel(reaction_motion_time_grid);
            
        reaction_time_indices = (car_2_change_time_start_index : ...
            (car_2_change_time_start_index + reaction_motion_time_grid_len - 1));
            
        opts.intersection_opts = struct(...
                field_names.x, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 1), ...
                field_names.y, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 2), ...
                field_names.phi, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 5), ...
                field_names.t, reaction_motion_time_grid ...
        );
                
        reaction_motion_sol = move_forward(reaction_motion_param, ...
                                           opts);
                
        reaction_motion_time_grid = reaction_motion_sol.x;
        reaction_motion_x_y_vx_vy_phi_grid = (reaction_motion_sol.y)';
    
        if print_intermediate_results
            print_car_motion_results('Прямолинейное движение 1-го автомобиля по реакции', ...
                                     reaction_motion_time_grid, ...
                                     reaction_motion_x_y_vx_vy_phi_grid);
        end
            
        if print_headers
            disp('FINISH 1st car reaction modelling');
        end
        
        total_time_grid = [total_time_grid, reaction_motion_time_grid(2 : end)];
        total_x_y_vx_vy_phi_grid = [ ...
            total_x_y_vx_vy_phi_grid; ...
            reaction_motion_x_y_vx_vy_phi_grid(2 : end, :) ...
        ];
        
        if (~isfield(reaction_motion_sol, 'xe')) || ...
            (numel(reaction_motion_sol.xe) == 0)
            
            % не произошло столкновение за время движения 1-го
            % автомобиля по реакции
                
            reaction_time_finish = reaction_motion_time_grid(end);
            left_after_reaction_time_mask = (car_2_time_grid >= reaction_time_finish);
            speedup_and_left_after_reaction_time_grid = car_2_time_grid(left_after_reaction_time_mask);
            
            if (numel(speedup_and_left_after_reaction_time_grid) > 1)    
                
                % время моделирования не закончилось при прямолинейном 
                % движении 1-го автомобиля по реакции
               
                % положения 1-го и 2-го автомобилей в момент окончания
                % движения 1-го автомобиля по реакции
                 
                [x, y, vx, vy, phi] = ...
                    get_car_x_y_vx_vy_phi(...
                        reaction_motion_x_y_vx_vy_phi_grid(end, :) ...
                );
                v = norm([vx, vy]);   
                    
                car_2_x_y_vx_vy_phi_at_reaction_finish_time = ...
                    car_2_x_y_vx_vy_phi_grid(car_2_time_grid == reaction_time_finish, :);

                car_2_x_at_reaction_finish_time = car_2_x_y_vx_vy_phi_at_reaction_finish_time(1);
                car_2_y_at_reaction_finish_time = car_2_x_y_vx_vy_phi_at_reaction_finish_time(2);
                car_2_phi_at_reaction_finish_time = car_2_x_y_vx_vy_phi_at_reaction_finish_time(5);
                
                car_2_frame_at_reaction_finish_time = get_car_frame(...
                    car_2_x_at_reaction_finish_time, ...
                    car_2_y_at_reaction_finish_time, ...
                    car_2_phi_at_reaction_finish_time ...
                );
            
                car_2_upper_left_corner_y_at_reaction_finish_time = ...
                    car_2_frame_at_reaction_finish_time(2, 2);
                
                
                if ((y <= car_2_upper_left_corner_y_at_reaction_finish_time) && ...
                    car_2_upper_left_corner_y_at_reaction_finish_time <= (y + car_length))
                    
                    % разгон требуется как попытка избежать столкновения
                    speedup_needed = true;
                
                    if print_headers
                        disp('START 1st car speedup modelling');
                    end
                    
% -------- разгон 1-го автомобиля после движения по реакции                                             
                   
                    speedup_motion_param = struct( ...
                        field_names.x, x, ...
                        field_names.y, y, ...
                        field_names.v, v, ...
                        field_names.phi, phi, ...
                        field_names.acc_tan, speedup_acc, ...
                        field_names.t, speedup_and_left_after_reaction_time_grid ...
                    );
                
                    v_before_speedup = v;
                    
                    opts = rmfield(opts, event_names.intersection_opts);
                
                    opts.collision_speedup_opts = struct( ...
                        field_names.x, car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 1), ...
                        field_names.y, car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 2), ...
                        field_names.phi, car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 5), ...
                        field_names.t, speedup_and_left_after_reaction_time_grid ...
                    );

                    speedup_sol = move_forward(speedup_motion_param, opts);  
                    
                    speedup_time_grid = speedup_sol.x;
                    speedup_x_y_vx_vy_phi_grid = (speedup_sol.y)';
            
                    if print_intermediate_results
                        print_car_motion_results('Разгон 1-го автомобиля', ...
                            speedup_time_grid, ...
                            speedup_x_y_vx_vy_phi_grid ...
                        );
                    end
        
                    if print_headers
                        disp('FINISH 1st car speedup modelling');
                    end
                    
                    opts = rmfield(opts, event_names.collision_speedup_opts);
                    
                    total_time_grid = [ ...
                        total_time_grid, ...
                        speedup_time_grid(2 : end) ...
                    ];
                                  
                    total_x_y_vx_vy_phi_grid = [ ...
                        total_x_y_vx_vy_phi_grid; ...
                        speedup_x_y_vx_vy_phi_grid(2 : end, :)
                    ];
                        
                    if (~isfield(speedup_sol, 'xe')) || ...
                       (numel(speedup_sol.xe) == 0) || ...
                       (speedup_sol.ie == 2)
                       
                        % не произошло столкновение при разгоне
                        % 1-го автомобиля
                       
                        speedup_time_finish = speedup_time_grid(end);
                        left_after_speedup_time_mask = (...
                            speedup_and_left_after_reaction_time_grid >= speedup_time_finish ...
                        );
                        left_after_speedup_time_grid = ...
                            speedup_and_left_after_reaction_time_grid(left_after_speedup_time_mask); 
                            
                        if numel(left_after_speedup_time_grid) > 1
                            % время моделирования не истекло при
                            % разгоне 1-го автомобиля
                                
% -------- моделирование движения 1-го автомобиля до конца времени
% моделирования
                            if print_headers
                                disp('START 1st car left motion modelling till t_end');
                            end
                            
                            [x, y, vx, vy, phi] = ...
                                get_car_x_y_vx_vy_phi(speedup_x_y_vx_vy_phi_grid(end, :));
    
                            v = norm([vx, vy]);
                            
% ------------ моделирование движения 1-го автомобиля по реакции 

                            if print_headers
                                disp('START reaction motion after speedup');
                            end
                            
                            reaction_after_speedup_time_start = left_after_speedup_time_grid(1);

                            reaction_after_speedup_time_mask = ( ...
                               (reaction_after_speedup_time_start <= left_after_speedup_time_grid) & ...
                               (left_after_speedup_time_grid <= (reaction_after_speedup_time_start + car_reaction_time)) ...
                            );
                            reaction_after_speedup_time_grid = ...
                                left_after_speedup_time_grid(reaction_after_speedup_time_mask);
        
                            reaction_after_speedup_param = struct(...
                                'x', x, ...
                                'y', y, ...
                                'v', v, ...
                                'phi', phi, ...
                                'acc_tan', 0, ...
                                't', reaction_after_speedup_time_grid ...
                            );
                        
                            reaction_after_speedup_sol = ...
                                move_forward(reaction_after_speedup_param, opts);
                            reaction_after_speedup_time_grid = reaction_after_speedup_sol.x;
                            reaction_after_speedup_x_y_vx_vy_phi_grid = (reaction_after_speedup_sol.y)';
                            
                            if print_intermediate_results
                                print_results( ...
                                    'Движениие 1-го автомобиля по реакции после разгона', ...
                                    reaction_after_speedup_time_grid, ...
                                    reaction_after_speedup_x_y_vx_vy_phi_grid ...
                                );
                            end
                            
                            if print_headers
                                disp('FINISH reaction_motion_after_speedup');
                            end
                            
                            total_time_grid = [ ...
                                total_time_grid, ...
                                reaction_after_speedup_time_grid(2 : end) ...
                            ];
                        
                            total_x_y_vx_vy_phi_grid = [ ...
                                total_x_y_vx_vy_phi_grid; ...
                                reaction_after_speedup_x_y_vx_vy_phi_grid(2 : end, :) ...
                            ];
                        
                            braking_time_start = ...
                                reaction_after_speedup_time_grid(end);
                            braking_and_left_after_reaction_time_mask = ...
                                (left_after_speedup_time_grid >= braking_time_start);
                            braking_and_left_after_reaction_time_grid = ...
                                left_after_speedup_time_grid(braking_and_left_after_reaction_time_mask);
                            
                            if (numel(braking_and_left_after_reaction_time_grid) > 0)
                            
                                % время моделирования не закончилось при
                                % движении 1-го авто по реакции после
                                % разгона
                                
% ------------ моделирование торможения 1-го автомобиля до начальной
% скорости перед разгоном

                                if print_headers
                                    disp('START 1st car braking');
                                end
                                
                                [x, y, vx, vy, phi] = ...
                                    get_car_x_y_vx_vy_phi(reaction_after_speedup_x_y_vx_vy_phi_grid(end, :));
    
                                v = norm([vx, vy]);
            
                                braking_param = struct( ...
                                    field_names.x, x, ...
                                    field_names.y, y, ...
                                    field_names.v, v, ...
                                    field_names.phi, phi, ...
                                    field_names.acc_tan, -speedup_acc, ...
                                    field_names.t, braking_and_left_after_reaction_time_grid ...
                                );
                        
                                opts.speed_opts = struct( ...
                                    'v_finish', v_before_speedup, ...
                                    'eps', 1e-2 ...
                                );
                            
                                braking_sol = move_forward(braking_param, opts);
                            
                                braking_time_grid = braking_sol.x;
                                braking_x_y_vx_vy_phi_grid = (braking_sol.y)';
                            
                                if print_intermediate_results
                                    print_results('Торможение 1-го автомобиля после разгона', ...
                                                  braking_time_grid, ...
                                                  braking_x_y_vx_vy_phi_grid...
                                    );
                                end
                            
                                opts = rmfield(opts, 'speed_opts');
                            
                                if print_headers
                                    disp('FINISH 1st car braking');
                                end
                                
                                total_time_grid = [ ...
                                    total_time_grid, ...
                                    braking_time_grid(2 : end)...
                                ];
                         
                                total_x_y_vx_vy_phi_grid = [ ...
                                    total_x_y_vx_vy_phi_grid; ...
                                    braking_x_y_vx_vy_phi_grid(2 : end, :) ...
                                ];
                        
                                braking_time_finish = braking_time_grid(end);
                                left_after_braking_time_mask = ...
                                    (braking_and_left_after_reaction_time_grid >= braking_time_finish);
                                left_after_braking_time_grid = ...
                                     braking_and_left_after_reaction_time_grid(...
                                         left_after_braking_time_mask ...
                                );
                            
                                if (numel(left_after_braking_time_grid) > 1)
                            
                                    % время моделирования не закончилось при
                                    % торможении 1-го автомобиля после движения
                                    % по реакции
                                
% ------------ моделирование прямолинейного движения 1-го автомобиля после
% торможения
                                
                                    [x, y, vx, vy, phi] = ...
                                        get_car_x_y_vx_vy_phi(braking_x_y_vx_vy_phi_grid(end, :));
                                    v = norm([vx, vy]);
                                
                                    left_after_braking_param = struct( ...
                                        field_names.x, x, ...
                                        field_names.y, y, ...
                                        field_names.v, v, ...
                                        field_names.phi, phi, ...
                                        field_names.acc_tan, acc_tan, ...
                                        field_names.t, left_after_braking_time_grid ...
                                    );
                                        
                                    left_after_braking_sol = ...
                                        move_forward(left_after_braking_param, opts);
                            
                                    left_after_braking_time_grid = left_after_braking_sol.x;
                                    left_after_braking_x_y_vx_vy_phi_grid = ...
                                        (left_after_braking_sol.y)';
            
                                    if print_intermediate_results
                                        print_results('Прямолинейное движение 1-го автомобиля после торможения', ...
                                                      left_after_braking_time_grid, ...
                                                      left_after_braking_x_y_vx_vy_phi_grid...
                                        );
                                    end
                            
                                    if print_headers
                                        disp('FINISH 1st car left motion modelling till t_end');
                                    end
                                    
                                    total_time_grid = [ ...
                                        total_time_grid, ...
                                        left_after_braking_time_grid(2 : end)...
                                    ];
                         
                                    total_x_y_vx_vy_phi_grid = [ ...
                                        total_x_y_vx_vy_phi_grid; ...
                                        left_after_braking_x_y_vx_vy_phi_grid(2 : end, :) ...
                                    ];
                            
                                end % не истекло время моделирования при торможении й-го автомобиля после движения по реакции
                                
                            end % время моделирования не закончилось при движении 1-го автомобиля по реакции после разгона
                            
                        end % время моделирования не истелко при разгоне 1-го автомобиля
                            
                    else
                        
                        % произошло столкновение при разгоне 1-го
                        % автомобиля
                        if print_headers
                            disp('Road accident happened');    
                        end
                        
                        road_accident_happened = true;
                            
                    end % произошло ли столкновение при разгоне 1-го автомобиля
            
                else
                    
                    % разгон как попытка избежать столкновения не
                    % требуется; движемся прямо с отслеживанием
                    % столкновения
                    
                    % обновляем параметры для отслеживания столкновения во
                    % время движения 1-го автомобиля после его движения по
                    % реакции
                    
                    
                    collision_watch_needed = false;
                    
                    if (y <= car_2_upper_left_corner_y_at_reaction_finish_time)
                        
                        collision_watch_needed = true;
                        
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.x, ...
                            car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 1) ...
                        );
                                      
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.y, ...
                            car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 2) ...
                        );
        
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.phi, ...
                            car_2_x_y_vx_vy_phi_grid(left_after_reaction_time_mask, 5) ...
                        );
                                      
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.t, ...
                            speedup_and_left_after_reaction_time_grid ...
                        );
                
                    else
                        opts = rmfield(opts, event_names.intersection_opts);
                    end
            
                    if print_headers
                        disp('START 1st car left motion modelling till t_end');
                    end
                    
                    left_after_reaction_param = ...
                        get_car_straight_motion_param_struct( ...
                            x, y, v, phi, acc_tan, speedup_and_left_after_reaction_time_grid ...
                        );     
                   
                    left_after_reaction_sol = ...
                        move_forward(left_after_reaction_param, opts);
                        
                    speedup_and_left_after_reaction_time_grid = left_after_reaction_sol.x;
                    left_after_reation_x_y_vx_vy_phi_grid = (left_after_reaction_sol.y)';
                    
                    if collision_watch_needed
                        
                        if isfield(left_after_reaction_sol, 'xe') && ...
                           (numel(left_after_reaction_sol.xe) > 0) 
                   
                            % произошло столкновение при движении 1-го
                            % автомобиля после его движения по реакции в
                            % отсутствие разгона
                        
                            if print_headers
                                disp("Road accident happened");
                            end
                            
                            road_accident_happened = true;
                        
                        end
                        
                        opts = rmfield(opts, event_names.intersection_opts);
                        
                    end
            
                    if print_intermediate_results
                        print_car_motion_results( ...
                            'Прямолинейное движение 1-го автомобиля после перестроения', ...
                            speedup_and_left_after_reaction_time_grid, ...
                            left_after_reation_x_y_vx_vy_phi_grid ...
                        );
                    end
                    
                    if print_headers
                        disp('FINISH 1st car left motion modelling till t_end');
                    end
                    
                    total_time_grid = [ ...
                        total_time_grid, ... 
                        speedup_and_left_after_reaction_time_grid(2 : end) ...
                    ];
                
                    total_x_y_vx_vy_phi_grid = [ ...
                        total_x_y_vx_vy_phi_grid; ...
                        left_after_reation_x_y_vx_vy_phi_grid(2 : end, :) ...
                    ];        
                   
                end
            
            else
                
                 % время моделирования истекло при движении 1-го
                 % автомобиля по реакции
                
            end
                
        else
            
            % произошло столкновение за время движения 1-го автомобиля
            % по реакции
            if print_headers
                disp('Road accident happened');
            end
            
            road_accident_happened = true;
                
        end    
        
    else
        
        % время моделирования закончилось при прямолинейном движении 1-го
        % автомобиля до начала перестроения 2-го
    
    end
    
    if print_headers
        disp('FINISH 1st car modelling: speedup while 2d car change');
    end
    
end