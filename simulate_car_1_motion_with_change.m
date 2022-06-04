function [total_time_grid, car_1_total_x_y_vx_vy_phi_grid, change_needed, road_accident_happened] = ...
    simulate_car_1_motion_with_change(...
        motion_start_param, ...
        curvature, ...
        car_2_time_grid, ...
        car_2_x_y_vx_vy_phi_grid ...
    )

% ---- моделирование движения 1-го автомобиля до конца времени моделирования
% ---- либо до столкновения

    print_opts = get_print_opts();
    print_intermediate_results = print_opts.print_intermediate_results;
    print_headers = print_opts.print_headers;
    
    if print_headers
        disp('START 1st car modelling: change while 2d car change');
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
    
    road_accident_happened = false;
    change_needed = false;
    
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
        
% -------- движение 1-го автомобиля по реакции

        if print_headers
            disp('START 1st car reaction motion');
        end
        
        [x, y, vx, vy, phi] = ...
            get_car_x_y_vx_vy_phi(straight_motion_x_y_vx_vy_phi_grid(end, :));
        
        reaction_motion_time_mask = (...
            (car_2_time_grid >= car_2_change_time_start) & ...
            (car_2_time_grid <= (car_2_change_time_start + car_reaction_time)) ...
        );
            
        v = norm([vx, vy]);
                
        reaction_motion_time_grid = ...
            car_2_time_grid(reaction_motion_time_mask);

        reaction_motion_param = ...
            get_car_straight_motion_param_struct(x, y, ...
                                                 v, phi, ...
                                                 acc_tan, ...
                                                 reaction_motion_time_grid);
                                                 
        % параметры для отслеживания столкновения за время движения 
        % 1-го автомобиля по реации
        time_change_index = find(car_2_time_grid == car_2_change_time_start, 1);
        reaction_motion_time_grid_len = numel(reaction_motion_time_grid);
            
        reaction_time_indices = (time_change_index : ...
            (time_change_index + reaction_motion_time_grid_len - 1));
            
        opts.intersection_opts = struct(...
            field_names.x, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 1), ...
            field_names.y, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 2), ...
            field_names.phi, car_2_x_y_vx_vy_phi_grid(reaction_time_indices, 5), ...
            field_names.t, reaction_motion_time_grid ...
        );
                
        % движение
        reaction_motion_sol = move_forward(reaction_motion_param, ...
                                           opts);
                
        reaction_motion_time_grid = reaction_motion_sol.x;
        reaction_motion_x_y_vx_vy_phi_grid = (reaction_motion_sol.y)';
    
        if print_intermediate_results
            
            print_car_motion_results( ...
                'Прямолинейное движение 1-го автомобиля по реакции', ...
                reaction_motion_time_grid, ...
                reaction_motion_x_y_vx_vy_phi_grid ...
            );
        
        end
            
        if print_headers
            disp('FINISH 1st car reaction modelling');
        end
        
        total_time_grid = [ ... 
            total_time_grid, ...
            reaction_motion_time_grid(2 : end) ...
        ];
    
        car_1_total_x_y_vx_vy_phi_grid = [ ...
            car_1_total_x_y_vx_vy_phi_grid; ...
            reaction_motion_x_y_vx_vy_phi_grid(2 : end, :) ...
        ];
       
        if ((~isfield(reaction_motion_sol, 'xe')) || ...
            (numel(reaction_motion_sol.xe) == 0))
            
            % не произошло столкновение за время движения 1-го
            % автомобиля по реакции
           
            reaction_motion_finish_time = reaction_motion_time_grid(end);
                
            change_and_left_time_grid = car_2_time_grid(...
                car_2_time_grid >= reaction_motion_finish_time ...
            );
           
            if (numel(change_and_left_time_grid) > 1)
                    
                % время моделирования не закончилось при прямолинейном 
                % движении 1-го автомобиля по реакции
               
                [x, y, vx, vy, phi] = ...
                   get_car_x_y_vx_vy_phi(...
                       reaction_motion_x_y_vx_vy_phi_grid(end, :) ...
                   );
               
               v = norm([vx, vy]);
               
               car_2_x_y_vx_vy_phi_at_reaction_finish_time = ...
               car_2_x_y_vx_vy_phi_grid(...
                   car_2_time_grid == reaction_motion_finish_time, :);
           
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
                
                car_1_bumper_y = y + car_length;
                distance_y = car_2_down_left_corner_y_at_reaction_finish_time - car_1_bumper_y;
               
                car_1_braking_acc = get_car_1_braking_motion_acc();
                max_braking_path = 0.5 * v^2 / -car_1_braking_acc;
                safe_dist = get_safe_distance();
               
                if ((((y + 0.5 * car_length) <= car_2_upper_left_corner_y_at_reaction_finish_time) && ...
                   (distance_y <= 0)) || ((distance_y > 0) && ...
                   (distance_y <= (max_braking_path + safe_dist))))
                    
                   % перестроение требуется
                   change_needed = true;
                   
                   if print_headers
                       disp('START 1st car lane change modelling');
                   end
                   
% -------- перестроение 1-го автомобиля после движения по реакции
                        
                   [turn_1_param, turn_2_param] = ...
                       get_car_1_change_param(v, curvature);
                        
                    turn_1_acc_rot = turn_1_param.acc_rot;
                    turn_1_R = turn_1_param.R;
                    turn_1_time = turn_1_param.time;
                        
                    turn_2_acc_rot = turn_2_param.acc_rot;
                    turn_2_R = turn_2_param.R;
                    turn_2_time = turn_2_param.time;
                        
                    % параметры первого поворота при перестроении 1-го
                    % автомобиля
    
                    turn_1_time_start =  change_and_left_time_grid(1);
                    turn_2_time_start = turn_1_time_start + turn_1_time;
                    turn_1_time_grid = change_and_left_time_grid(...
                        change_and_left_time_grid <= turn_2_time_start ...
                    );
    
                    turn_1_motion_param = struct(field_names.x, x, ...
                                                 field_names.y, y, ...
                                                 field_names.v, v, ...
                                                 field_names.phi, phi, ...
                                                 field_names.acc_rot, turn_1_acc_rot, ...
                                                 field_names.R, turn_1_R, ...
                                                 field_names.t, turn_1_time_grid);
                
                    % параметры второго поворота при перестроении 1-го
                    % автомобиля
    
                    change_time_finish = turn_2_time_start + turn_2_time;
                    turn_2_time_grid = change_and_left_time_grid(...
                        (turn_2_time_start <= change_and_left_time_grid) & ...
                        (change_and_left_time_grid <= change_time_finish) ...
                    );
                    
                    turn_2_motion_param = struct(field_names.acc_rot, turn_2_acc_rot, ...
                                                 field_names.R, turn_2_R, ...
                                                 field_names.t, turn_2_time_grid);
                                                 
                    % обновляем параметры для отслеживания столкновения
                    % во время перестроения 1-го автомобиля
    
                    change_time_mask = (...
                        (reaction_motion_finish_time <= car_2_time_grid) & ...
                        (car_2_time_grid <= change_time_finish) ...
                    );
                                  
                    opts.intersection_opts = setfield( ...
                        opts.intersection_opts, ...
                        field_names.x, ...
                        car_2_x_y_vx_vy_phi_grid(change_time_mask, 1) ...
                    );
                                      
                    opts.intersection_opts = setfield( ...
                        opts.intersection_opts, ...
                        field_names.y, ...
                        car_2_x_y_vx_vy_phi_grid(change_time_mask, 2) ...
                    );
        
                    opts.intersection_opts = setfield( ...
                        opts.intersection_opts, ...
                        field_names.phi, ...
                        car_2_x_y_vx_vy_phi_grid(change_time_mask, 5) ...
                    );
                                      
                    opts.intersection_opts = setfield(...
                        opts.intersection_opts, ...
                        field_names.t, ...
                        car_2_time_grid(change_time_mask) ...
                    );
                                                   
                    % моделирование перестроения 1-го автомобиля

                    change_sol = ...
                        lane_change(turn_1_motion_param, ...
                                    turn_2_motion_param, ...
                                    opts, ...
                                    false);
        
                    change_time_grid = change_sol.x;
                    change_x_y_vx_vy_phi_grid = (change_sol.y)';
                    change_time_finish = change_time_grid(end);
                        
                    if print_intermediate_results
                        print_car_motion_results('Перестроение 1-го автомобиля', ...
                                                 change_time_grid, ...
                                                 change_x_y_vx_vy_phi_grid);
                    end
                    
                    if print_headers
                        disp('FINISH 1st car lane change modelling');
                    end
                    
                    total_time_grid = [total_time_grid, change_time_grid(2 : end)];
                    car_1_total_x_y_vx_vy_phi_grid = [ ...
                        car_1_total_x_y_vx_vy_phi_grid; ...
                        change_x_y_vx_vy_phi_grid(2 : end, :)
                    ];
                
                    opts = rmfield(opts, event_names.intersection_opts);
                        
                    if (~isfield(change_sol, 'xe')) || ...
                       (numel(change_sol.xe) == 0)
                       
                        % не произошло столкновение при перестроении
                        % 1-го автомобиля
                       
                        left_motion_time_mask = (...
                            change_and_left_time_grid >= change_time_finish ...
                        );
                        left_motion_time_grid = change_and_left_time_grid(left_motion_time_mask); 
                        
                            
                        if numel(left_motion_time_grid) > 1
                            % время моделирования не истекло при
                            % перестроении 1-го автомобиля
                                
% -------- моделирование движения 1-го автомобиля до конца времени
% моделирования
                            if print_headers
                                disp('START 1st car left motion modelling till t_end');
                            end
                            
                            % параметры прямолинейного движения
            
                            [x, y, vx, vy, phi] = ...
                                get_car_x_y_vx_vy_phi(change_x_y_vx_vy_phi_grid(end, :));
    
                            v = norm([vx, vy]);

                            left_motion_param = struct(field_names.x, x, ...
                                                       field_names.y, y, ...
                                                       field_names.v, v, ...
                                                       field_names.phi, phi, ...
                                                       field_names.acc_tan, acc_tan, ...
                                                       field_names.t, left_motion_time_grid);
                                                                                        
                            % моделирование прямолинейного движения 1-го
                            % автомобиля

                            left_motion_sol = move_forward(left_motion_param, ...
                                                           opts);
                            
                            left_motion_time_grid = left_motion_sol.x;
                            left_motion_x_y_vx_vy_phi_grid = (left_motion_sol.y)';
            
                            if print_intermediate_results
                                print_results('Прямолинейное движение 1-го автомобиля после перестроения', ...
                                              left_motion_time_grid, ...
                                              left_motion_x_y_vx_vy_phi_grid);
                            end
                            
                            if print_headers
                                disp('FINISH 1st car left motion modelling till t_end');
                            end
                            
                            total_time_grid = [total_time_grid, ...
                                left_motion_time_grid(2 : end)...
                            ];
                         
                            car_1_total_x_y_vx_vy_phi_grid = [car_1_total_x_y_vx_vy_phi_grid; ...
                                left_motion_x_y_vx_vy_phi_grid(2 : end, :)
                            ];
                            
                        end
                            
                    else
                        
                        % произошло столкновение при перестроении 1-го
                        % автомобиля
                        if print_headers
                            disp('Road accident happened');    
                        end
                        
                        road_accident_happened = true;
                            
                    end
            
                else
                    
                    % манёвр не требуется 
            
                    if print_headers
                        disp('START 1st car left motion modelling till t_end');
                    end
                    
                    left_motion_time_grid = ...
                        car_2_time_grid(car_2_time_grid >= reaction_motion_finish_time);

                    left_motion_param = ...
                        get_car_straight_motion_param_struct( ...
                            x, y, v, phi, acc_tan, left_motion_time_grid);     
                   
                    left_motion_sol = move_forward(left_motion_param, opts);
                        
                    left_motion_time_grid = left_motion_sol.x;
                    left_motion_x_y_vx_vy_phi_grid = (left_motion_sol.y)';
            
                    if print_intermediate_results
                        print_car_motion_results( ...
                            'Прямолинейное движение 1-го автомобиля после перестроения', ...
                            left_motion_time_grid, ...
                            left_motion_x_y_vx_vy_phi_grid ...
                        );
                    end
            
                    if print_headers
                        disp('FINISH 1st car left motion modelling till t_end');
                    end
                    
                    total_time_grid = [ ...
                        total_time_grid, ... 
                        left_motion_time_grid(2 : end) ...
                    ];
                
                    car_1_total_x_y_vx_vy_phi_grid = [ ...
                        car_1_total_x_y_vx_vy_phi_grid; ...
                        left_motion_x_y_vx_vy_phi_grid(2 : end, :) ...
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
                
            opts = rmfield(opts, event_names.intersection_opts);
                
        end
        
    else
        
        % время моделирования закончилось при прямолинейном движении 1-го
        % автомобиля до начала перестроения 2-го
    
    end
    
    if print_headers
        disp('FINISH 1st car modelling: change while 2d car change');
    end
     
end