% Подбор подходящих манёвров 1-го автомобиля (торможение / разгон /
% перестроение) при перестроении 1-го автомобиля для различных начальных 
% парметров движения  

clear;
clc;

disp('START program');

%% входные данные

    % начальные параметры движения автомобилей

[car_1_start_motion_param, car_2_start_motion_param] = get_cars_start_motion_param();

car_1_x_start = car_1_start_motion_param.x;
car_1_y_start = car_1_start_motion_param.y;
car_1_v_grid = car_1_start_motion_param.v_grid;
car_1_acc_tan = car_1_start_motion_param.acc_tan;

car_2_x_start = car_2_start_motion_param.x;
car_2_y_grid = car_2_start_motion_param.y_grid;
car_2_v_start = car_2_start_motion_param.v;
car_2_acc_tan = car_2_start_motion_param.acc_tan;

car_1_curvature_grid_len = 5; % количество точек в сетке кривизны при повороте 1-го автомобиля  

car_1_change_acceptable_v_dist_min_max_curv = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                                  % minimum car_1_curvature, maximum car_1_cruvature, 
                                                  % при которых не будет столкновения
                                                  % при перестроении 1-го автомобиля в
                                                  % ответ на перестроение 2-го

car_1_speedup_acceptable_v_dist = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                     % при которых не будет столкновения
                                     % при перестроении 1-го автомобиля в
                                     % ответ на перестроение 2-го
                                     
car_1_braking_acceptable_v_dist = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                     % при которых не будет столкновения
                                     % при перестроении 1-го автомобиля в
                                     % ответ на перестроение 2-го

%% Моделирование

car_1_v_grid_len = numel(car_1_v_grid);
car_2_y_grid_len = numel(car_2_y_grid);

results = cell(car_1_v_grid_len, car_2_y_grid_len);

braking = struct();
speedup = struct();
change = struct();

% Поиск допустимых параметров (при которых удастся избежать столкновения
% путём перестроения и орисовка моделирования

for car_1_v_grid_idx = 1 : car_1_v_grid_len
    
    for car_2_y_grid_idx = 1 : car_2_y_grid_len
        
% Моделирование
        
        car_1_v_start = car_1_v_grid(car_1_v_grid_idx);
        car_2_y_start = car_2_y_grid(car_2_y_grid_idx);
        
% ---- моделирование движения 2-го автомобиля до конца времени
% моделирования
    
        car_2_motion_start_param = struct('x', car_2_x_start, ...
                                          'y', car_2_y_start, ...
                                          'v', car_2_v_start, ...
                                          'acc_tan', car_2_acc_tan);
    
        [car_2_time_grid, car_2_x_y_vx_vy_phi_grid] = ...
            simulate_car_2_motion(car_2_motion_start_param);
        
% ---- моделирование движения 1-го автомобиля до конца времени моделирования
% ---- либо до столкновения

        car_1_motion_start_param = struct('x', car_1_x_start, ...
                                          'y', car_1_y_start, ...
                                          'v', car_1_v_start, ...
                                          'acc_tan', car_1_acc_tan);

        % торможение 
        [~, ~, braking_needed, braking_accident_happened] = ...
            simulate_car_1_motion_with_braking(car_1_motion_start_param, ...
                                               car_2_time_grid, ...
                                               car_2_x_y_vx_vy_phi_grid ...
                                               );
        
        % разгон
        [~, ~, speedup_needed, speedup_accident_happened] = ...
            simulate_car_1_motion_with_speedup(car_1_motion_start_param, ...
                                               car_2_time_grid, ...
                                               car_2_x_y_vx_vy_phi_grid ...
                                               );
                                           
        % перестроение 
        
        car_1_max_curvature = calc_max_curvature(car_1_v_start);
        car_1_curvature_grid = linspace(0, car_1_max_curvature, car_1_curvature_grid_len);
        car_1_min_curvature = car_1_max_curvature;
        
        for car_1_curvature_grid_index = 2 : car_1_curvature_grid_len

            car_1_curvature = car_1_curvature_grid(car_1_curvature_grid_index);
            
            [~, ~, change_needed, change_accident_happened] = ...
                simulate_car_1_motion_with_change(car_1_motion_start_param, ...
                                                  car_1_curvature, ...
                                                  car_2_time_grid, ...
                                                  car_2_x_y_vx_vy_phi_grid ...
                                                  );
            
            if ~change_accident_happened
                car_1_min_curvature = car_1_curvature;
                break;
            end
            
        end
        
% ---- Оценка допустимости параметров в зависимости от того, произошло ли
% столкновение 

        braking.ok = false;
        braking.needed = braking_needed;
        speedup.ok = false;
        speedup.needed = speedup_needed;
        change.ok = false;
        change.needed = change_needed;
                                        
        if ~braking_accident_happened && braking_needed
               
            car_1_braking_acceptable_v_dist = [...
                car_1_braking_acceptable_v_dist; ...
                car_1_v_start, car_2_y_start - car_1_y_start ...
            ]; 
       
            braking.ok = true;
        end
       
       if ~speedup_accident_happened && speedup_needed
       
           car_1_speedup_acceptable_v_dist = [ ...
               car_1_speedup_acceptable_v_dist; ...
               car_1_v_start, car_2_y_start - car_1_y_start, ...
           ];
       
           speedup.ok = true;
           
       end
       
       if ~change_accident_happened && change_needed
           
           car_1_change_acceptable_v_dist_min_max_curv = [...
               car_1_change_acceptable_v_dist_min_max_curv; ...
               car_1_v_start, ...
               car_2_y_start - car_1_y_start, ... 
               car_1_min_curvature, ...
               car_1_max_curvature ...
           ]; 
       
           change.ok = true;
           change.curvature_range = [car_1_min_curvature, car_1_max_curvature];
           
       end
       
       results{car_1_v_grid_idx, car_2_y_grid_idx} = ...
           {braking, speedup, change};      
           
    end
    
end
        
print_acceptable_parameters(car_1_braking_acceptable_v_dist, ...
                            car_1_speedup_acceptable_v_dist, ...
                            car_1_change_acceptable_v_dist_min_max_curv ...
                            );

distance_grid = car_2_y_grid - car_1_y_start;
                        
print_acceptable_maneuvers(car_1_v_grid, ...
                           distance_grid, ...
                           results ...
                          );
                      
acceptable_values = [true, false];

for braking_acceptable_idx = 1 : 2
    for speedup_acceptable_idx = 1 : 2
        for change_acceptable_idx = 1 : 2
            
            acceptable_maneuvers = struct( ...
                'braking', acceptable_values(braking_acceptable_idx), ...
                'speedup', acceptable_values(speedup_acceptable_idx), ...
                'change', acceptable_values(change_acceptable_idx) ... 
            );
        
            print_parameters_for_specific_maneuvers(car_1_v_grid, ...
                                                     distance_grid, ...
                                                     results, ...
                                                     acceptable_maneuvers ...
            );
                                                
        end
    end 
end
                                   
disp('FINISH program');