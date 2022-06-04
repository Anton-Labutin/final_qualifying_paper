function [total_time_grid, car_1_total_x_y_vx_vy_phi_grid, ...
          braking_needed, road_accident_happened] = ...
    simulate_car_1_motion_with_braking(motion_start_param, ...
                                       car_2_time_grid, ...
                                       car_2_x_y_vx_vy_phi_grid...
                                       )

% ---- моделирование движения 1-го автомобиля до конца времени моделирования
% ---- либо до столкновения

    print_opts = get_print_opts();
    print_intermediate_results = print_opts.print_intermediate_results;
    print_headers = print_opts.print_headers;

    if print_headers
        disp('START 1st car motion: braking while 2d car change');
    end

    motion_time_param = get_motion_time_param();
    car_2_change_time_start = motion_time_param.change;

    x_start = motion_start_param.x;
    y_start = motion_start_param.y;
    v_start = motion_start_param.v;
    acc_tan = motion_start_param.acc_tan;
    
    car_1_braking_acc = get_car_1_braking_motion_acc();

    car_param = get_car_param();
    car_length = car_param.length;
    car_reaction_time = car_param.time_reaction;
    
    road_accident_happened = false;
    braking_needed = false;
    
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
    car_1_total_x_y_vx_vy_phi_grid = straight_motion_x_y_vx_vy_phi_grid;
                     
    if (car_2_time_grid(~straight_motion_time_mask) > 0)
        
        % время моделирования не закончилось при прямолинейном движении
        % 1-го автомобиля до начала перестроения 2-го
        
        [x, y, vx, vy, phi] = ...
            get_car_x_y_vx_vy_phi(straight_motion_x_y_vx_vy_phi_grid(end, :));    
                    
% -------- движение 1-го автомобиля по реакции

        if print_headers
            disp('START 1st car reaction motion');
        end
            
        car_1_reaction_time_finish = car_2_change_time_start + car_reaction_time;
        
        reaction_motion_time_mask = (...
                (car_2_time_grid >= car_2_change_time_start) & ...
                (car_2_time_grid <= car_1_reaction_time_finish) ...
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
        car_1_total_x_y_vx_vy_phi_grid = [car_1_total_x_y_vx_vy_phi_grid; ...
                                                reaction_motion_x_y_vx_vy_phi_grid(2 : end, :)];
        
        if (~isfield(reaction_motion_sol, 'xe')) || ...
            (numel(reaction_motion_sol.xe) == 0)
            
            % не произошло столкновение за время движения 1-го
            % автомобиля по реакции
            
            car_1_reaction_time_finish = reaction_motion_time_grid(end);
            braking_and_left_after_reaction_time_mask = (car_2_time_grid >= car_1_reaction_time_finish);
            braking_and_left_after_reaction_time_grid = car_2_time_grid(braking_and_left_after_reaction_time_mask);
            
            if (numel(braking_and_left_after_reaction_time_grid) > 1)
                    
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
                    car_2_x_y_vx_vy_phi_grid(car_2_time_grid == car_1_reaction_time_finish, :);

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
                car_2_down_left_corner_y_at_reaction_finish_time = ...
                    car_2_frame_at_reaction_finish_time(2, 1);
                
                max_braking_path = 0.5 * v^2 / -car_1_braking_acc;
                safe_dist = get_safe_distance();
                
                car_1_bumper_y = y + car_length;
                distance_y = car_2_down_left_corner_y_at_reaction_finish_time - car_1_bumper_y;
                
                if (((car_1_bumper_y < car_2_upper_left_corner_y_at_reaction_finish_time) && ...
                   ( distance_y <= 0)) || ((distance_y > 0) && ...
                   (distance_y <= (max_braking_path + safe_dist))))
               
                    % торможение требуется как попытка избежать
                    % столкновения
                    
                    braking_needed = true;
                
                    if print_headers
                        disp('START 1st car braking modelling');
                    end
                    
% -------- торможение 1-го автомобиля после движения по реакции                                             
                   
                    braking_motion_param = struct(field_names.x, x, ...
                                                  field_names.y, y, ...
                                                  field_names.v, v, ...
                                                  field_names.phi, phi, ...
                                                  field_names.acc_tan, car_1_braking_acc, ...
                                                  field_names.t, braking_and_left_after_reaction_time_grid);
                    
                    opts = rmfield(opts, event_names.intersection_opts);
                    
                    opts.collision_braking_opts = struct( ...
                        'car_2_x_y_vx_vy_phi_grid', ...
                            car_2_x_y_vx_vy_phi_grid(braking_and_left_after_reaction_time_mask, :), ...
                        field_names.t, braking_and_left_after_reaction_time_grid, ...
                        field_names.acc_tan, car_1_braking_acc ...
                    );
                
                    braking_sol = move_forward(braking_motion_param, opts);
                    
                    braking_time_grid = braking_sol.x;
                    braking_x_y_vx_vy_phi_grid = (braking_sol.y)';
                    
            
                    if print_intermediate_results
                        print_car_motion_results('Торможение 1-го автомобиля', ...
                            braking_time_grid, ...
                            braking_x_y_vx_vy_phi_grid ...
                        );
                    end
        
                    if print_headers
                        disp('FINISH 1st car braking modelling');
                    end
                    
                    opts = rmfield(opts, 'collision_braking_opts');
                    
                    
                    total_time_grid = [total_time_grid, braking_time_grid(2 : end)];
                    car_1_total_x_y_vx_vy_phi_grid = [ ...
                        car_1_total_x_y_vx_vy_phi_grid; ...
                        braking_x_y_vx_vy_phi_grid(2 : end, :)
                    ];
                
                        
                    if (~isfield(braking_sol, 'xe')) || ...
                       (numel(braking_sol.xe) == 0) || ...
                       (braking_sol.ie(1) == 2) 
                       
                        % не произошло столкновение при торможении
                        % 1-го автомобиля
                        
                        braking_time_finish = braking_time_grid(end);
                        left_after_braking_time_mask = (...
                            car_2_time_grid >= braking_time_finish ...
                        );
                        left_after_braking_time_grid = ...
                            car_2_time_grid(left_after_braking_time_mask); 
                       
                            
                        if numel(left_after_braking_time_grid) > 1
                            % время моделирования не истекло при
                            % торможении 1-го автомобиля
                                
% -------- моделирование движения 1-го автомобиля до конца времени
% моделирования
                            if print_headers
                                disp('START 1st car left motion modelling till t_end');
                            end
                            
                            % параметры прямолинейного движения
            
                            [x, y, vx, vy, phi] = ...
                                get_car_x_y_vx_vy_phi(braking_x_y_vx_vy_phi_grid(end, :));
    
                            v = norm([vx, vy]);
                            
                            left_after_braking_motion_param = struct( ...
                                field_names.x, x, ...
                                field_names.y, y, ...
                                field_names.v, v, ...
                                field_names.phi, phi ...
                            );
                            
                            car_2_x_y_vx_vy_phi_grid_after_braking = ...
                                car_2_x_y_vx_vy_phi_grid(left_after_braking_time_mask, :);
                            
                            [ left_after_braking_time_grid, ...
                              left_after_braking_x_y_vx_vy_phi_grid, ...
                              accident_after_braking ...
                            ] = straight_motion_after_braking(...
                                    left_after_braking_time_grid, ...
                                    left_after_braking_motion_param, ...
                                    car_2_x_y_vx_vy_phi_grid_after_braking ...
                                );
                            
                            
                            road_accident_happened = accident_after_braking;
                                        
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
                         
                            car_1_total_x_y_vx_vy_phi_grid = [ ...
                                car_1_total_x_y_vx_vy_phi_grid; ...
                                left_after_braking_x_y_vx_vy_phi_grid(2 : end, :) ...
                            ];
                            
                        end % время моделирования не истелко при торможении 1-го автомобиля
                            
                    else
                        
                        % произошло столкновение при торможении 1-го
                        % автомобиля  
                        
                        if print_headers
                            disp('Road accident happened');
                        end
                        
                        road_accident_happened = true;
                            
                    end
            
                else
                    
                    % торможение как попытка избежать столкновения не
                    % требуется; продолжаем прямолинейное движение с
                    % возможным отслеживанием столкновения
            
                    if print_headers
                        disp('START 1st car left motion modelling till t_end');
                    end
                    
                    collision_watch_needed = false;
                    
                    if (y <= car_2_upper_left_corner_y_at_reaction_finish_time)
                    
                        % требуется отследить возможное столкновение
                        collision_watch_needed = true; 
                    
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.x, ...
                            car_2_x_y_vx_vy_phi_grid(braking_and_left_after_reaction_time_mask, 1) ...
                        );
                                      
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.y, ...
                            car_2_x_y_vx_vy_phi_grid(braking_and_left_after_reaction_time_mask, 2) ...
                        );
        
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.phi, ...
                            car_2_x_y_vx_vy_phi_grid(braking_and_left_after_reaction_time_mask, 5) ...
                        );
                                      
                        opts.intersection_opts = setfield( ...
                            opts.intersection_opts, ...
                            field_names.t, ...
                            braking_and_left_after_reaction_time_grid ...
                        );
                    else
                        opts = rmfield(opts, event_names.intersection_opts);
                    end

                    left_after_reaction_param = ...
                        get_car_straight_motion_param_struct( ...
                            x, y, v, phi, acc_tan, braking_and_left_after_reaction_time_grid ...
                        );     
                   
                    left_after_reaction_sol = ...
                        move_forward(left_after_reaction_param, opts);
                        
                    left_after_reaction_time_grid = left_after_reaction_sol.x;
                    left_after_reaction_x_y_vx_vy_phi_grid = (left_after_reaction_sol.y)';
            
                    if print_intermediate_results
                        print_car_motion_results( ...
                            'Прямолинейное движение 1-го автомобиля после движения по реакции', ...
                            left_after_reaction_time_grid, ...
                            left_after_reaction_x_y_vx_vy_phi_grid ...
                        );
                    end
                    
                    if collision_watch_needed
                        
                        if (isfield(left_after_reaction_sol, 'xe') && ...
                            (numel(left_after_reaction_sol.xe) > 0))
                        
                            road_accident_happened = true;
                        
                        end
                        
                        opts = rmfield(opts, event_names.intersection_opts);
                    end
            
                    if print_headers
                        disp('FINISH 1st car left motion modelling till t_end');
                    end
                    
                    total_time_grid = [ ...
                        total_time_grid, ... 
                        left_after_reaction_time_grid(2 : end) ...
                    ];
                    
                    car_1_total_x_y_vx_vy_phi_grid = [ ...
                        car_1_total_x_y_vx_vy_phi_grid; ...
                        left_after_reaction_x_y_vx_vy_phi_grid(2 : end, :) ...
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
        disp('FINISH 1st car motion: braking while 2d car change');
    end
    
end