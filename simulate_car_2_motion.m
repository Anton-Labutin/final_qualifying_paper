function [total_time_grid, total_x_y_vx_vy_phi_grid] = ...
            simulate_car_2_motion(motion_start_param)

% моделирование движения 2-го автомобиля

    print_opts = get_print_opts();
    print_intermediate_results = print_opts.print_intermediate_results;
    print_headers = print_opts.print_headers;
    
    if print_headers
        disp('START 2d car motion');
    end
    
    motion_time = get_motion_time_param();

    time_start = motion_time.start;    % время начала моделирования
    time_finish = motion_time.finish;  % время конца моделирования
    time_change = motion_time.change;  % время начала перестроения
    time_step = motion_time.step;      % шаг по временной сетке [time_start, time_finish]
        
    x_start = motion_start_param.x;
    y_start = motion_start_param.y;   
    v_start = motion_start_param.v;
    acc_tan = motion_start_param.acc_tan;
    
    [turn_1_param, turn_2_param] = get_car_2_change_param(v_start);
    
    turn_1_acc_rot = turn_1_param.acc_rot;
    turn_1_time = turn_1_param.time;
    turn_1_R = turn_1_param.R;
    
    turn_2_acc_rot = turn_2_param.acc_rot;
    turn_2_time = turn_2_param.time;
    turn_2_R = turn_2_param.R;
    
    opts.ode_opts = get_start_ode_opts();
    
    field_names = get_field_names();
    
    total_time_grid = [];
    total_x_y_vx_vy_phi_grid = [];
    
% ---- прямолинейное движение до начала перестроения 

    if print_headers
        disp('START 2d car straight motion untill lane change');
    end
    
% -------- начальные параметры движения 

    straight_motion_time_grid = (time_start : time_step : time_change);
                                        
    straight_motion_param = ...
        get_car_straight_motion_param_struct(x_start, ...
                                             y_start, ...
                                             v_start, ...
                                             0, ...
                                             acc_tan, ...
                                             straight_motion_time_grid);
                                         
    opts.time_limit_opts = struct(field_names.t, time_finish, ...
                                  field_names.eps, get_eps());
                                         
% -------- моделирование прямолинейного движения до начала перестроения
% -------- 2-го автомобиля либо истечения времени моделирования 
    
    straight_motion_sol = move_forward(straight_motion_param, ...
                                       opts);
        
    total_time_grid = straight_motion_sol.x;                            
    total_x_y_vx_vy_phi_grid = (straight_motion_sol.y)';
        
    if print_intermediate_results
        print_car_motion_results('Прямолинейное движение 2-го автомобиля до начала перестроения', ...
                                 total_time_grid, ...
                                 total_x_y_vx_vy_phi_grid);
    end
    
    if print_headers
        disp('FINISH 2d straight motion untill lane change');
    end    
    
    if (~isfield(straight_motion_sol, 'xe')) || ...
       (numel(straight_motion_sol.xe) == 0)   
            
       % время моделирования не истекло при прямолинейном движении 2-го
       % автомобиля до начала перестроения
    
% ---- перестроение 2го автомобиля
    
        if print_headers
            disp('START 2d car lane change modelling');
        end
        
% -------- начальные параметры 2-го автомобиля перед перестроением 
     
        [x, y, vx, vy, phi] = ...
            get_car_x_y_vx_vy_phi(total_x_y_vx_vy_phi_grid(end, :));
        
% -------- параметры для первого поворота при перестроении 2го
% автомобиля
                
        v = norm([vx, vy]);
        turn_1_time_start = total_time_grid(end);
        turn_1_time_finish = turn_1_time_start + turn_1_time;
        turn_1_time_step = min(time_step, turn_1_time / 10);
        
        turn_1_time_grid = (turn_1_time_start : turn_1_time_step : turn_1_time_finish); 
        diff_1 = turn_1_time_finish - turn_1_time_grid(end);
        if (diff_1 < turn_1_time_step)
            turn_1_time_grid = [turn_1_time_grid, turn_1_time_finish];
        end

% -------- параметры второго поворота при перестроении 2-го автомобиля
                             
        turn_2_time_start = turn_1_time_finish;
        turn_2_time_finish = turn_2_time_start + turn_2_time;
        turn_2_time_step = min(time_step, turn_2_time / 10);
        
        turn_2_time_grid = (turn_2_time_start : turn_2_time_step : turn_2_time_finish);
        diff_2 = turn_2_time_finish - turn_2_time_grid(end); 
        if (diff_2 < turn_2_time_step)
            turn_2_time_grid = [turn_2_time_grid, turn_2_time_finish];
        end
        
        diff = min(diff_1, diff_2);
        opts.ode_opts = odeset(opts.ode_opts, 'InitialStep', diff, 'MaxStep', diff);
        
        turn_1_motion_param = struct(field_names.x, x, ...
                                     field_names.y, y, ...
                                     field_names.v, v, ...
                                     field_names.phi, phi, ...
                                     field_names.acc_rot, turn_1_acc_rot, ...
                                     field_names.R, turn_1_R, ...
                                     field_names.t, turn_1_time_grid);
        
        turn_2_motion_param = struct(field_names.acc_rot, turn_2_acc_rot, ...
                                     field_names.R, turn_2_R, ...
                                     field_names.t, turn_2_time_grid);
             
% -------- моделирование перестроения 2го автомобиля (частичное, если истечёт время моделирования)

        change_motion_sol = lane_change(turn_1_motion_param, ...
                                        turn_2_motion_param, ...
                                        opts, ...
                                        false);
    
        change_motion_time_grid = change_motion_sol.x;
        change_motion_x_y_vx_vy_phi_grid = (change_motion_sol.y)';
        
        total_time_grid = [total_time_grid, change_motion_time_grid(2 : end)];
        total_x_y_vx_vy_phi_grid = [total_x_y_vx_vy_phi_grid; ...
            change_motion_x_y_vx_vy_phi_grid(2 : end, :)];
              
        if print_intermediate_results
            print_car_motion_results('Перестроение 2-го автомобиля', ...
                                     change_motion_time_grid, ...
                                     change_motion_x_y_vx_vy_phi_grid);
        end
        
        if print_headers
            disp('FINISH 2d car lane change modelling');
        end
            
        if (~isfield(change_motion_sol, 'xe')) || ...
            (numel(change_motion_sol.xe) == 0) 
           
            % время модeлирования не закончилось при перестроении 2-го автомобиля

            if print_headers
                disp('START 2d car left motion modelling till time_finish');
            end
            
% ---- моделирование прямолинейного движения 2го автомобиля до
% конца времени моделирования 

% -------- параметры движения

            [x, y, vx, vy, phi] = ...
                get_car_x_y_vx_vy_phi(change_motion_x_y_vx_vy_phi_grid(end, :));

            left_motion_time_start = change_motion_time_grid(end);
        
            v = norm([vx, vy]);
            
            left_motion_time_grid = ... 
                (left_motion_time_start : time_step : time_finish);
            
            if (left_motion_time_grid(end) < time_finish)
                left_motion_time_grid = [ ...
                    left_motion_time_grid, time_finish ...
                ];
            end
            
            left_motion_param = ...
                get_car_straight_motion_param_struct(x, y, v, phi, acc_tan, ...
                    left_motion_time_grid);
                                            
% -------- моделирование движения 2-го автомобиля до конца времени моделирования

            left_motion_sol = move_forward(left_motion_param, opts);
        
            left_motion_time_grid = left_motion_sol.x;
            left_motion_x_y_vx_vy_phi_grid = (left_motion_sol.y)';
            
            if print_intermediate_results
                print_car_motion_results('Прямолинейное движение 2-го автомобиля после перестроения', ...
                                         left_motion_time_grid, ...
                                         left_motion_x_y_vx_vy_phi_grid);
            end
            
            total_time_grid = [total_time_grid, left_motion_time_grid(2 : end)];
            total_x_y_vx_vy_phi_grid = [total_x_y_vx_vy_phi_grid; ...
                left_motion_x_y_vx_vy_phi_grid(2 : end, :)];
    
            if print_headers
                disp('FINISH 2d car left motion modelling till t_end');
            end
            
        end

    end

    if print_headers
        disp('FINISH 2d car motion');
    end
    
end