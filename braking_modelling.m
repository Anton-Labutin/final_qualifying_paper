% Моделирование торможения управляемого автомобиля при перестроении 2го
% автомобиля

clear;
clc;

disp('START program');

%% входные данные

print_opts = get_print_opts();
print_intermediate_results = print_opts.print_intermediate_results;
print_headers = print_opts.print_headers;
print_acceptable_parameters = print_opts.print_acceptable_parameters;

if print_headers
    disp('START reading input');
end

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

car_1_braking_acceptable_v_dist = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                     % при которых не будет столкновения
                                     % при перестроении 1-го автомобиля в
                                     % ответ на перестроение 2-го
                         
draw_opts = get_draw_opts();
draw_motion_needed = draw_opts.draw_motion_needed;
draw_motion_accident_needed = draw_opts.draw_motion_accident_needed;
draw_motion_ok_needed = draw_opts.draw_motion_ok_needed;
motion_video_needed = draw_opts.motion_video_needed;

% фигура для моделирования
figure = get_figure();

if print_headers
    disp('FINISH reading input');
end

%% Моделирование

car_1_v_grid_len = numel(car_1_v_grid);
car_2_y_grid_len = numel(car_2_y_grid);


% Поиск допустимых параметров (при которых удастся избежать столкновения
% путём перестроения и орисовка моделирования

for car_1_v_grid_idx = 1 : car_1_v_grid_len
    
    for car_2_y_grid_idx = 1 : car_2_y_grid_len
        
% Моделирование
        
        car_1_v_start = car_1_v_grid(car_1_v_grid_idx);
        car_2_y_start = car_2_y_grid(car_2_y_grid_idx);
        
        if print_headers
            disp(...
                ['START modelling: car_1_v = ', num2str(car_1_v_start), ...
                ' м/с, cars_distance = ', num2str(car_2_y_start - car_1_y_start), ...
                ' м'] ...
            );
        end
    
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

        [car_1_time_grid, car_1_x_y_vx_vy_phi_grid, ...
            braking_needed, road_accident_happened] = ...
            simulate_car_1_motion_with_braking(car_1_motion_start_param, ...
                                               car_2_time_grid, ...
                                               car_2_x_y_vx_vy_phi_grid ...
                                               );
           
        cur_time_grid = car_1_time_grid;
        cur_time_grid_len = numel(car_1_time_grid);
        size(car_2_x_y_vx_vy_phi_grid)
        cur_car_2_x_y_vx_vy_phi_grid = car_2_x_y_vx_vy_phi_grid(...
            1 : cur_time_grid_len, : ...
        );
        
        if print_intermediate_results
              
            cur_time_grid_indices = (1 : 1 : cur_time_grid_len)';
                 
            print_results( ... 
                [ cur_time_grid_indices, cur_time_grid' ], ...
                [ cur_time_grid_indices, car_1_x_y_vx_vy_phi_grid ], ...
                [ cur_time_grid_indices, cur_car_2_x_y_vx_vy_phi_grid ] ...
            );
            
        end
                                             
        if draw_motion_needed
            
            if ((~road_accident_happened) && draw_motion_ok_needed) || ...
                (road_accident_happened && draw_motion_accident_needed)
                    
% ---- отрисовка
           
                if print_headers
                    disp('START drawing');
                end
                
                param = struct('car_1_v_start', car_1_v_start, ...
                               'start_distance', car_2_y_start - car_1_y_start ...
                               );
                    
                draw_motion(figure, ...
                            param, ...
                            cur_time_grid, ...
                            car_1_x_y_vx_vy_phi_grid, ...
                            cur_car_2_x_y_vx_vy_phi_grid, ...
                            road_accident_happened, ...
                            motion_video_needed ...
                );
        
                if print_headers
                    disp('FINISH drawing');    
                end
                
            end
               
       end
                                              
       if ~road_accident_happened
               
           car_1_braking_acceptable_v_dist = [...
               car_1_braking_acceptable_v_dist; ...
               car_1_v_start, car_2_y_start - car_1_y_start ...
           ]; 
           
       end
                                                  
       if print_headers
           disp( ...
               ['FINISH modelling: car_1_v = ', ...
               num2str(car_1_v_start), ...
               ' м/с, distance = ', ...
               num2str(car_2_y_start - car_1_y_start), ... 
               ' м, road_accident_happened: ', ...
               num2str(road_accident_happened)] ...
           );
       end
           
    end
    
end
      
if print_acceptable_parameters
    disp('Acceptable parameters: ');
    disp('car_1_v (м/с) | distance (м)');
    disp(car_1_braking_acceptable_v_dist);
end

disp('FINISH program');