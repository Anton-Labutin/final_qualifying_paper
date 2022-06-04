% Моделирование торможения управляемого автомобиля при перестроении 2го
% автомобиля

disp('START program');

clear;
clc;

%% входные данные

disp('START reading input');

    % параметры области моделирования

field = get_field_param();

field_width = field.width;     % ширина области моделирования
field_height = field.height;   % высота области моделирования

    % параметры времени моделирования

motion_time = get_motion_time_param();

time_start = motion_time.start;    % время начала моделирования
time_finish = motion_time.finish;  % время конца моделирования
time_change = motion_time.change;  % время начала перестроения
time_step = motion_time.step;      % шаг по временной сетке [time_start, time_finish]

    % параметр погрешности вычислений

eps = 1e-3;

    % параметры дороги

road = get_road_param(field.width);

lane_width = road.lane_width;
lane_cnt = road.lane_cnt;
road_left_side_x = road.left_side_x;
road_friction_coef = road.friction_coef;
road_friction_ratio = road.friction_ratio;
road_turn_margin = road.turn_margin;

    % технические параметры автомобилей

car = get_car_param();

car_length = car.length;
car_width = car.width;
car_mass = car.mass;
car_time_reaction = car.time_reaction;

    % начальные параметры движения автомобилей

[car_1_start_motion_param, car_2_start_motion_param] = get_cars_start_motion_param(field_height, road);

car_1_x_start = car_1_start_motion_param.x;
car_1_y_start = car_1_start_motion_param.y;
car_1_v_grid = car_1_start_motion_param.v_grid;
car_1_acc_tan = car_1_start_motion_param.acc_tan;

car_2_x_start = car_2_start_motion_param.x;
car_2_y_grid = car_2_start_motion_param.y_grid;
car_2_v_start = car_2_start_motion_param.v;
car_2_acc_tan = car_2_start_motion_param.acc_tan;


car_1_curvature_grid_len = 10; % количество точек в сетке кривизны при повороте 1-го автомобиля
rot_angle = pi / 6; % угол, на который натянута первая дуга окружности при перестроении 2-го автомобиля

car_1_braking_acceptable_v_dist = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                             % при которых не будет столкновения при
                             % торможении 1-го автомобиля в ответ на
                             % перестроение 2-го
                             
car_1_speedup_acceptable_v_dist = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                           % при которых не будет столкновения
                                           % при разгоне 1-го автомобиля в
                                           % ответ на перестроение 2-го
                                           
car_1_change_acceptable_v_dist_curv = []; % параметры car_1_v_start, distance = car_2_y_start - car_1_y_start, 
                                           % minimum car_2_curvature, при
                                           % которых не будет столкновения
                                           % при перестроении 1-го автомобиля в
                                           % ответ на перестроение 2-го
                                           
% фигура для визуализации моделирвания                  
fig = figure('Units', 'normalized', ...
             'OuterPosition', [0 0 1 1], ...
             'Position', [0 0 1 1]);
         
field_names = get_field_names();
event_names = get_event_names();

% car_1_time_indices_and_events = [];
% car_2_time_indices_and_events = [];

disp('FINISH reading input');

%% Моделирование

opts.ode_opts = odeset('RelTol', eps, ...
                       'AbsTol', eps, ...
                       'InitialStep', time_step, ...
                       'MaxStep', time_step);

car_1_v_grid_len = numel(car_1_v_grid);
car_2_y_grid_len = numel(car_2_y_grid);

road_param = struct('friction_coef', road_friction_coef, ...
                    'friction_ratio', road_friction_ratio ...
                   );

car_dimensions = struct('length', car_length, ...
                        'width', car_width);
               
% параметры торможения 1-го автомобиля при перестроении 2-го
car_1_braking_param = get_car_1_braking_motion_param(road_param);
car_1_braking_v_finish = car_1_braking_param.v_finish; % конечная скорость, до которой тормозим
car_1_braking_acc = car_1_braking_param.acc;           % ускорение при торможении

road_param.turn_margin = road_turn_margin;

% максимальная кривизна перестроения 2-го автомобиля
car_2_max_curvature = calc_max_curvature(road_param, car_2_v_start);

% параметры осей для визуализации моделирования
ax = get_axes(fig, field);


% Поиск допустимых параметров для каждого манёвра

for car_1_v_grid_idx = 1 : car_1_v_grid_len
    
    for car_2_y_grid_idx = 1 : car_2_y_grid_len
        
% Моделирование

        happened_road_accident = false;
        
        car_1_v_start = car_1_v_grid(car_1_v_grid_idx);
        car_2_y_start = car_2_y_grid(car_2_y_grid_idx);
        
        disp(...
            ['START modelling: car_1_v = ', num2str(car_1_v_start), ...
             ' м/с, cars_distance = ', num2str(car_2_y_start - car_1_y_start), ...
             ' м'] ...
        );
        
% ---- прямолинейное движение обоих автомобилей до начала перестроения 2го автомобиля    

% -------- начальные параметры движения 

        straight_motion_time_grid = [time_start, time_change];

        car_1_straight_motion_param = ...
            get_car_straight_motion_param_struct(car_1_x_start, ...
                                                 car_1_y_start, ...
                                                 car_1_v_start, ...
                                                 0, ...
                                                 car_1_acc_tan, ...
                                                 straight_motion_time_grid);
                                        
        car_2_straight_motion_param = ...
            get_car_straight_motion_param_struct(car_2_x_start, ...
                                                 car_2_y_start, ...
                                                 car_2_v_start, ...
                                                 0, ...
                                                 car_2_acc_tan, ...
                                                 straight_motion_time_grid);
           
        opts.time_limit_opts = struct(field_names.t, time_finish, ...
                                      field_names.eps, eps);
                                         
% -------- моделирование прямолинейного движения двух автомобилей до начала
% -------- перестроения 2го автомобиля либо истечения времени моделирования 
    
        disp('START straight motion untill lane change');
        
        car_1_straight_motion_sol = move_forward(car_1_straight_motion_param, ...
                                                 opts);
                                                 
        car_2_straight_motion_param = setfield(car_2_straight_motion_param, ...
                                               field_names.t, ...
                                               car_1_straight_motion_sol.x);
                                               
        opts = rmfield(opts, event_names.time_limit_opts);

        car_2_straight_motion_sol = move_forward(car_2_straight_motion_param, ...
                                                 opts);
        
        opts.time_limit_opts = struct(field_names.t, time_finish, ...
                                      field_names.eps, eps);
        
        total_motion_time_grid = car_2_straight_motion_sol.x;                            % итоговая временная сетка
        car_1_total_motion_x_y_vx_vy_phi_omega_grid = (car_1_straight_motion_sol.y)'; % итоговая сетка позиций 1го автомобиля
        car_2_total_motion_x_y_vx_vy_phi_omega_grid = (car_2_straight_motion_sol.y)'; % итоговая сетка позиций 2го автомобиля
        
        print_results(total_motion_time_grid', ...
                      car_1_total_motion_x_y_vx_vy_phi_omega_grid, ...
                      car_2_total_motion_x_y_vx_vy_phi_omega_grid);
        
%         car_1_time_indices_and_events = [car_1_time_indices_and_events; ...
%                                          [numel(total_motion_time_grid), 'прямолинейное движение до перестроения']];
%                                      
%         car_2_time_indices_and_events = [car_2_time_indices_and_events; ...
%                                          [numel(total_motion_time_grid), 'прямолинейное движение до перестроения']];
                                     
        disp('FINISH straight motion untill lane change');
        
        if (~isfield(car_1_straight_motion_sol, 'xe')) || ...
           (numel(car_1_straight_motion_sol.xe) == 0)   
            
           % время моделирования не истекло при движении до начала
           % перестроения 2-го автомобиля
        
            disp('modelling not finished yet');
    
% ---- перестроение 2го автомобиля и реакция 1го автомобиля
    
% --------- перестроение 2го автомобиля

% ------------ начальные параметры 2го автомобиля перед перестроением 
     
            [car_2_x, car_2_y, car_2_vx, car_2_vy, car_2_phi, car_2_omega] = ...
                get_car_x_y_vx_vy_phi_omega(car_2_total_motion_x_y_vx_vy_phi_omega_grid(end, :));
        
% ---------------- параметры для первого поворота при перестроении 2го
% автомобиля
                
            car_2_v = norm([car_2_vx, car_2_vy]);
            car_2_turn_1_acc_rot = 0;
            % car_2_turn_1_R = road.lane_width / (3 * (1 + cos(pi - rot_angle))); % по оси х переместиться на lane_width / 3, повернувшиь на угол rot_angle;
            car_2_turn_1_R = lane_width / (2 * (1 + cos(pi - rot_angle)));
                
            if abs(car_2_turn_1_R - 1 / car_2_max_curvature) < 1e-1
                car_2_turn_1_R = 1 / car_2_max_curvature;
            end
            
            car_2_turn_1_time = rot_angle * car_2_turn_1_R / car_2_v;

            car_2_turn_1_time_start = total_motion_time_grid(end);
            car_2_turn_1_time_finish = car_2_turn_1_time_start + car_2_turn_1_time;
    
            car_2_turn_1_motion_param = struct(field_names.x, car_2_x, ...
                                               field_names.y, car_2_y, ...
                                               field_names.v, car_2_v, ...
                                               field_names.phi, car_2_phi, ...
                                               field_names.acc_rot, car_2_turn_1_acc_rot, ...
                                               field_names.R, car_2_turn_1_R, ...
                                               field_names.t, [car_2_turn_1_time_start, car_2_turn_1_time_finish]);

% ---------------- параметры для прямолинейного движения при перестроении
% 2-го автомобиля

%             car_2_straight_motion_acc_tan = 0;
% 
%             car_2_straight_motion_time = road.lane_width / (3 * v_0_2 * sin(rot_angle));
% 
%             car_2_straight_motion_time_start = car_2_turn_1_time_start + car_2_turn_1_time;
%             car_2_straight_motion_time_finish = car_2_straight_motion_time_start + car_2_straight_motion_time;
%     
%             car_2_straight_motion_param = struct(field_names.acc_tan, car_2_straight_motion_acc_tan, ...
%                                                  field_names.t, [car_2_straight_motion_time_start, car_2_straight_motion_time_finish]);

% ---------------- параметры второго поворота при перестроении 2-го
% автомобиля
                             
            car_2_turn_2_acc_rot = 0;
            % car_2_turn_2_R = road.lane_width / (3 * (1 - cos(rot_angle)));
            car_2_turn_2_R = lane_width / (2 * (1 - cos(rot_angle)));
            
            if abs(car_2_turn_2_R - 1 / car_2_max_curvature) < 1e-1
                car_2_turn_2_R = 1 / car_2_max_curvature;
            end
            
            car_2_turn_2_time =  car_2_turn_2_R * rot_angle / car_2_v_start;

            % car_2_turn_2_time_start = car_2_straight_motion_time_start + car_2_straight_motion_time;
            car_2_turn_2_time_start = car_2_turn_1_time_start + car_2_turn_1_time;
            car_2_turn_2_time_finish = car_2_turn_2_time_start + car_2_turn_2_time;
            
            car_2_turn_2_motion_param = struct(field_names.acc_rot, car_2_turn_2_acc_rot, ...
                                               field_names.R, car_2_turn_2_R, ...
                                               field_names.t, [car_2_turn_2_time_start, car_2_turn_2_time_finish]);
             
            % car_2_turn_2_motion_param = struct(field_names.acc_rot, car_2_turn_2_acc_rot, ...
            %                                   'time_start', car_2_turn_2_time_start, ...
            %                                   'x_finish', road.left_side_pos + road.lane_width / 2, ...
            %                                   'max_curvature', car_1_max_curvature);
            
% ------------ моделирование перестроения 2го автомобиля (частичное, если истечёт время моделирования)

            disp('START 2d car lane change modelling');
    
            car_2_change_motion_sol = lane_change(car_2_turn_1_motion_param, ...
                                                  car_2_turn_2_motion_param, ...
                                                  opts, ...
                                                  false);
    
            car_2_change_motion_time_grid = car_2_change_motion_sol.x;
            car_2_change_motion_x_y_vx_vy_phi_omega_grid = (car_2_change_motion_sol.y)';
                            
            disp('Временная сетка: ');
            disp(car_2_change_motion_time_grid');
            disp('Параметры движения 2го автомобиля: ');
            disp(car_2_change_motion_x_y_vx_vy_phi_omega_grid);
            
            car_2_time_grid = car_2_change_motion_time_grid;
            car_2_x_y_vx_vy_phi_omega_grid = car_2_change_motion_x_y_vx_vy_phi_omega_grid;
    
            disp('FINISH 2d car lane change modelling');
            
            
            if (~isfield(car_2_change_motion_sol, 'xe')) || ...
               (numel(car_2_change_motion_sol.xe) == 0) 
           
               % время модeлирования не закончилось при перестроении 2-го автомобиля

                disp('modelling not finished yet');
% -------- моделирование прямолинейного движения 2го автомобиля до
% конца времени моделирования 

% ------------ параметры движения

                [car_2_x, car_2_y, car_2_vx, car_2_vy, car_2_phi, car_2_omega] = ...
                    get_car_x_y_vx_vy_phi_omega(car_2_change_motion_x_y_vx_vy_phi_omega_grid(end, :));

                car_2_left_motion_time_start = car_2_change_motion_time_grid(end);
        
                car_2_v_start = norm([car_2_vx, car_2_vy]);
        
            
                car_2_left_motion_param = struct(field_names.x, car_2_x, ...
                                                 field_names.y, car_2_y, ...
                                                 field_names.v, car_2_v, ...
                                                 field_names.phi, car_2_phi, ...
                                                 field_names.acc_tan, car_2_start_motion_param.acc_tan, ...
                                                 field_names.t, [car_2_left_motion_time_start, motion_time.finish]);
                                        

% ------------ моделирование движения 2-го автомобиля до конца времени моделирования

                disp('START 2d car left motion modelling till t_end');

                car_2_left_motion_sol = move_forward(car_2_left_motion_param, opts);
        
                car_2_left_motion_time_grid = car_2_left_motion_sol.x;
                car_2_left_motion_x_y_vx_vy_phi_omega_grid = (car_2_left_motion_sol.y)';
            
                disp('Временная сетка: ');
                disp(car_2_left_motion_time_grid');
                disp('Параметры движения 2го автомобиля: ');
                disp(car_2_left_motion_x_y_vx_vy_phi_omega_grid);
   
                car_2_time_grid = [car_2_time_grid, ...
                                   car_2_left_motion_time_grid(2 : end)];
            
                car_2_x_y_vx_vy_phi_omega_grid = [car_2_x_y_vx_vy_phi_omega_grid; ...
                                                  car_2_left_motion_x_y_vx_vy_phi_omega_grid(2 : end, :)];
    
                disp('FINISH 2d car left motion modelling till t_end');
            
            end
               
            
% -------- движение 1го автомобиля
                
            [car_1_x, car_1_y, car_1_vx, car_1_vy, car_1_phi, car_1_omega] = ...
                get_car_x_y_vx_vy_phi_omega(car_1_total_motion_x_y_vx_vy_phi_omega_grid(end, :));

            if car_1_y <= ...
               (car_2_total_motion_x_y_vx_vy_phi_omega_grid(end, 2) + car_dimensions.length)
               % манёвр требуется
                    
% ------------ движение за время реакции

% ---------------- начальные параметры движения

                car_1_reaction_motion_time_mask = (car_2_time_grid <= (car_2_time_grid(1) + car.time_reaction));
                car_1_reaction_motion_time_grid = car_2_time_grid(car_1_reaction_motion_time_mask);
                car_1_reaction_motion_time_grid_len = numel(car_1_reaction_motion_time__grid);
    
                car_1_v = norm([car_1_vx, car_1_vy]);

                car_1_reaction_motion_param = struct(field_names.x, car_1_x, ...
                                                     field_names.y, car_1_y, ...
                                                     field_names.v, car_1_v, ...
                                                     field_names.phi, car_1_phi, ...
                                                     field_names.acc_tan, car_1_start_motion_param.acc_tan, ...
                                                     field_names.t, car_1_reaction_motion_time_grid);
                                                 
                opts = rmfield(opts, event_names.time_limit_opts);

                % параметры для отслеживания столкновения за время реакции 1го
                % автомобиля
    
                opts.intersection_opts = struct(field_names.x, car_2_x_y_vx_vy_phi_omega_grid(1 : car_1_reaction_motion_time_grid_len, 1), ...
                                                field_names.y, car_2_x_y_vx_vy_phi_omega_grid(1 : car_1_reaction_motion_time_grid_len, 2), ...
                                                field_names.phi, car_2_x_y_vx_vy_phi_omega_grid(1 : car_1_reaction_motion_time_grid_len, 5), ...
                                                field_names.length, car_length, ...
                                                field_names.width, car_width, ...
                                                field_names.t, car_1_reaction_motion_time_grid);

% ---------------- моделирование движения за время реакции (либо меньше, если время моделирования истекло или произошло столкновение)

                disp('START 1st car reaction modelling');

                car_1_reaction_motion_sol = move_forward(car_1_reaction_motion_param, opts);
                
                car_1_reaction_motion_time_grid = car_1_reaction_motion_sol.x;
                car_1_reaction_motion_x_y_vx_vy_phi_omega_grid = (car_1_reaction_motion_sol.y)';
    
                disp('Временная сетка: ');
                disp(car_1_reaction_motion_time_grid');
                disp('Параметры движения 1го автомобиля: ');
                disp(car_1_reaction_motion_x_y_vx_vy_phi_omega_grid);
    
                disp('FINISH 1st car reaction modelling');
                
                
% ------------ перестроение 1-го автомобиля после движения по реакции

                % максимальная кривизна при перестроении 1-го автомобиля в ответ на
                % перестроение 2-го
                car_1_max_curvature = calc_max_curvature(road_param, car_1_v_start);
                car_1_curvature_grid = linspace(0, car_1_max_curvature, car_1_curvature_grid_len);
                car_1_curvature_grid_len = numel(car_1_curvature_grid);
                
                % ищем наименьшую кривизну поворота, при которой не будет
                % столкновения
                
                for i = 1 : car_1_curvature_grid_len
                    
                    car_1_curvature = car_1_curvature_grid(i);
                    
                    [car_1_x, car_1_y, car_1_vx, car_1_vy, car_1_phi, car_1_omega] = ...
                        get_car_x_y_vx_vy_phi_omega(car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(end, :));
    
                    car_1_change_and_left_time_grid = [...
                        car_1_reaction_motion_time_grid(end), ...
                        car_2_time_grid(~car_1_reaction_motion_time_mask)...
                    ];
                
% ---------------- параметры первого поворота при перестроении 1-го
% автомобиля
                
                    car_1_v = norm([car_1_vx, car_1_vy]);
                    car_1_turn_1_acc_rot = 0;
                    car_1_turn_1_R = 1 / car_1_curvature; 
                    car_1_turn_1_time = rot_angle * car_1_turn_1_R / car_1_v;
    
                    car_1_turn_1_time_start =  car_1_change_and_left_time_grid(1);
                    t_grid_turn_1 = car_1_change_and_left_time_grid(...
                        car_1_change_and_left_time_grid <= (car_1_turn_1_time_start + car_1_turn_1_time));
    
                    car_1_turn_1_motion_param = struct(field_names.x, car_1_x, ...
                                                       field_names.y, car_1_y, ...
                                                       field_names.v, car_1_v, ...
                                                       field_names.phi, car_1_phi, ...
                                                       field_names.acc_rot, car_1_turn_1_acc_rot, ...
                                                       field_names.R, car_1_turn_1_R, ...
                                                       field_names.t, car_1_turn_1_time_grid);
                
% ---------------- параметры второго поворота при перестроении 1-го
% автомобиля
                             
                    car_1_turn_2_acc_rot = 0;
                    % радиус поворота направо, при котором 1-й автомобиль
                    % выедет на середину полосы параллельно обочине
                    car_1_turn_2_R = (car_1_x_start - ...
                        car_1_turn_1_R * (1 - cos(rot_angle)) - ...
                        road_left_side_x - 0.5 * lane_width) / ...
                        (1 + cos(rot_angle));
                    
                    if abs(car_1_turn_2_R - 1 / car_1_max_curvature) < 1e-1
                        car_1_turn_2_R = 1 / car_1_max_curvature;
                    end
                    
                    car_1_turn_2_time =  car_1_turn_2_R * rot_angle / car_1_v;
    
                    car_1_turn_2_time_start = car_1_turn_1_time_grid(end);
                    
                    car_1_turn_2_time_grid = car_1_change_and_left_time_grid(...
                        (car_1_turn_2_time_start <= car_1_change_and_left_time_grid) & ...
                        (car_1_change_and_left_time_grid <= (car_1_turn_2_time_start + ...
                                                             car_1_turn_2_time)) ...
                    );
    
                    car_1_turn_2_motion_param = struct(field_names.acc_rot, car_1_turn_2_acc_rot, ...
                                                       field_names.R, car_1_turn_2_R, ...
                                                       field_names.t, car_1_turn_2_time_grid);
                
% ---------------- обновляем параметры для отслеживания столкновения во время перестроения 1-го автомобиля
    
                    car_1_change_time_mask = ((car_1_reaction_motion_time_grid(end) <= car_2_time_grid) & ...
                                      (car_2_time_grid <= car_1_turn_2_time_grid(end)));
    
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.x, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(car_1_change_time_mask, 1));
                                      
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.y, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(car_1_change_time_mask, 2));
        
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.phi, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(car_1_change_time_mask, 5));
                                      
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.t, ...
                                                      car_2_time_grid(car_1_change_time_mask));
                                                   
                                                  
% ------------ моделирование перестроения 1-го автомобиля (частичное, если произойдёт столкновение)

                    disp('START 1st car lane change modelling');

                    car_1_change_sol = ...
                        lane_change(car_1_turn_1_motion_param, ...
                                    car_1_turn_2_motion_param, ...
                                    opts, ...
                                    false);
        
                    car_1_change_time_grid = car_1_change_sol.x;
                    car_1_change_x_y_vx_vy_phi_omega_grid = (car_1_change_sol.y)';
                
                    disp('Временная сетка: ');
                    disp(car_1_change_time_grid');
                    disp('Параметры движения 1го автомобиля: ');
                    disp(car_1_change_x_y_vx_vy_phi_omega_grid);
    
                    disp('FINISH 1st car lane change modelling');
                    
                    
                    if (~isfield(car_1_change_sol, 'xe')) ...
                       || (numel(car_1_change)sol.xe) == 0) ...
                       || (car_1_change_sol.ie(1) == 2) 
                   
                       % 1-й автомобиль перестроился и не врезался
                       
% -------- моделирование движения 1го автомобиля до конца времени
% моделирования
            
                      disp('road accident not happened');
                      
                      % параметры прямолинейного движения
            
                      [car_1_x, car_1_y, car_1_vx, car_1_vy, car_1_phi, car_1_omega] = ...
                          get_car_x_y_vx_vy_phi_omega(car_1_change_x_y_vx_vy_phi_omega_grid(end, :));

                      car_1_left_motion_time_mask = (car_2_time_grid >= car_1_change_time_grid(end));
                      car_1_left_motion_time_grid = car_2_time_grid(car_1_left_motion_time_mask);
    
                      car_1_v = norm([car_1_vx, car_1_vy]);

                      car_1_left_motion_param = struct(field_names.x, car_1_x, ...
                                                field_names.y, car_1_y, ...
                                                field_names.v, car_1_v, ...
                                                field_names.phi, car_1_phi, ...
                                                field_names.acc_tan, car_1_acc_tan, ...
                                                field_names.t, car_1_left_motion_time_grid);
                                     
                      opts = rmfield(opts, event_names.intersection_opts);
                      
                      % моделирование прямолинейного движения 1-го
                      % автомобиля
            
                      disp('START 1st car left motion modelling till t_end');

                      car_1_left_motion_sol = ...
                          move_forward(car_1_left_motion_param, opts);
                      car_1_left_motion_time_grid = car_1_left_motion_sol.x;
                      car_1_left_motion_x_y_vx_vy_phi_omega_grid = (car_1_left_motion_sol.y)';
            
                      disp('Временная сетка: ');
                      disp(car_1_left_motion_time_grid');
                      disp('Параметры движения 1го автомобиля: ');
                      disp(car_1_left_motion_x_y_vx_vy_phi_omega_grid);
   
                      
                      car_1_total_motion_time_grid_prev = car_1_total_motion_time_grid;
                      car_1_total_motion_time_grid = (...
                          [car_1_total_motion_time_grid, ...
                           car_1_reaction_motion_time_grid(2 : end), ...
                           car_1_change_time_grid(2 : end), ...
                           car_1_left_motion_time_grid(2 : end)...
                          ] ...
                      )';
                  
                      car_1_total_motion_x_y_vx_vy_phi_omega_grid_prev = car_1_total_motion_x_y_vx_vy_phi_omega_grid;
                      car_1_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                          car_1_total_motion_x_y_vx_vy_phi_omega_gridl; ...
                          car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(2 : end, :); ...
                          car_1_change_x_y_vx_vy_phi_omega_grid(2 : end, :); ...
                          car_1_left_motion_x_y_vx_vy_phi_omega_grid(2 : end, :) ...
                      ];
                    
                      car_2_total_motion_x_y_vx_vy_phi_omega_grid_prev = car_2_total_motion_x_y_vx_vy_phi_omega_grid;
                      car_2_total_motion_x_y_vx_vy_phi_omega_grid = [...
                          car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                          car_2_x_y_vx_vy_phi_omega_grid(2 : end, :) ...
                      ]; 
                  
                      car_1_change_acceptable_v_dist_curv = [...
                          car_1_change_acceptable_v_dist_curv; ...
                          car_1_v, car_2_y_start - car_1_y_start, car_1_curvature ...
                      ];
                                            
                      disp('FINISH 1st car left motion modelling till t_end');
                      

% -------- отрисовка 
                      
        else 
            happened_road_accident = true;
            disp('road accident happened');
            
            t_grid_total = ([t_grid_total, ...
                        t_grid_reaction(2 : end), ...
                        t_grid_change_1(2 : end)])';

            x_y_vx_vy_phi_omega_grid_1_total = [x_y_vx_vy_phi_omega_grid_1_total; ...
                                            x_y_vx_vy_phi_omega_grid_reaction_1(2 : end, :); ...
                                            x_y_vx_vy_phi_omega_grid_change_1(2 : end, :)];
                                        
            x_y_vx_vy_phi_omega_grid_2_total = [x_y_vx_vy_phi_omega_grid_2_total; ...
                                            x_y_vx_vy_phi_omega_grid_2(2 : ...
                                                (numel(t_grid_reaction) + numel(t_grid_change_1) - 1), :)];
        end
                
                end
                
                
                if (~isfield(car_1_reaction_motion_sol, 'xe')) ||....
                   (numel(car_1_reaction_motion_sol.xe) == 0) % время моделирования не истекло и не произошло столкновение за время реакции

                    disp('modelling not finished yet');
                    
% ------------ торможение 

% ---------------- параметры торможения

                    [car_1_x, car_1_y, car_1_vx, car_1_vy, car_1_phi, car_1_omega] = ...
                        get_car_x_y_vx_vy_phi_omega(car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(end, :));


                    car_1_braking_motion_time_grid = [car_1_reaction_motion_time_grid(end), ...
                                                      car_2_time_grid(~car_1_reaction_motion_time_mask)];
                                                  
                    car_1_v = norm([car_1_vx, car_1_vy]);

                    car_1_braking_motion_param = struct(field_names.x, car_1_x, ...
                                                        field_names.y, car_1_y, ...
                                                        field_names.v, car_1_v, ...
                                                        field_names.phi, car_1_phi, ...
                                                        field_names.acc_tan, car_1_braking_param.acc, ...
                                                        field_names.t, car_1_braking_motion_time_grid);
                             
                    % обновляем параметры для отслеживания столкновения во время
                    % торможения 1го автомобиля
                    
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.x, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(numel(car_1_reaction_motion_time_grid) : end, 1));
                                      
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.y, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(numel(car_1_reaction_motion_time_grid) : end, 2));
        
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.phi, ...
                                                      car_2_x_y_vx_vy_phi_omega_grid(numel(car_1_reaction_motion_time_grid) : end, 5));
                                      
                    opts.intersection_opts = setfield(opts.intersection_opts, ...
                                                      field_names.t, ...
                                                      car_1_braking_motion_time_grid);
        
                    % параметры для отслеживания момента достижения конечной скорости
        
                    opts.motion_opts = struct(field_names.v, car_1_braking_param.v_finish, ...
                                              field_names.eps, eps); 
                                          
% ---------------- моделирование торможения до наступления хотя бы одного
% из событий: 
% 1. Время моделирования закончилось (учтено в t_grid_2)
% 2. Скорость машины стала = 0
% 3. Произошло столкновение машин

                    disp('START 1st car braking modelling');

                    car_1_braking_motion_sol = move_forward(car_1_braking_motion_param, opts);    
                    car_1_braking_motion_time_grid = car_1_braking_motion_sol.x;
                    car_1_braking_motion_x_y_vx_vy_phi_omega_grid = (car_1_braking_motion_sol.y)';
            
                    disp('Временная сетка: ');
                    disp(car_1_braking_motion_time_grid');
                    disp('Параметры движения 1го автомобиля: ');
                    disp(car_1_braking_motion_x_y_vx_vy_phi_omega_grid);
        
                    disp('FINISH 1st car braking modelling');
     
                    opts = rmfield(opts, event_names.motion_opts);
                    
                    
        
                    
                            if (~isfield(car_1_braking_motion_sol, 'xe')) || ...
                       (numel(car_1_braking_motion_sol.xe) == 0) || ...
                       (car_1_braking_motion_sol.ie(1) == 2) % 1й автомобиль остановился и не врезался
            
% ------------ моделирование движения 1го автомобиля до конца времени
% моделирования
            
                        disp('road accident not happened');
            
                        [car_1_x, car_1_y, car_1_vx, car_1_vy, car_1_phi, car_1_omega] = ...
                            get_cur_pos(car_1_braking_motion_x_y_vx_vy_phi_omega_grid(end, :));

                        car_1_left_motion_time_mask = (car_2_time_grid >= car_1_braking_motion_time_grid(end));
                        car_1_left_motion_time_grid = car_2_time_grid(car_1_left_motion_time_mask);
    
                        car_1_v = norm([car_1_vx, car_1_vy]);

                        car_1_left_motion_param = struct(field_names.x, car_1_x, ...
                                                         field_names.y, car_1_y, ...
                                                         field_names.v, car_1_v, ...
                                                         field_names.phi, car_1_phi, ...
                                                         field_names.acc_tan, car_1_start_motion_param.acc_tan, ...
                                                         field_names.t, car_1_left_motion_time_grid);
                        
        
                        disp('START 1st car left motion modelling till t_end');

                        car_1_left_motion_sol = move_forward(car_1_left_motion_param, opts);
                        
                        car_1_left_motion_time_grid = car_1_left_motion_sol.x;
                        car_1_left_motion_x_y_vx_vy_phi_omega_grid = (car_1_left_motion_sol.y)';
            
                        disp('Временная сетка: ');
                        disp(car_1_left_motion_time_grid');
                        disp('Параметры движения 1го автомобиля: ');
                        disp(car_1_left_motion_x_y_vx_vy_phi_omega_grid);
   
                        total_motion_time_grid = (...
                            [total_motion_time_grid, ...
                             car_1_reaction_motion_time_grid(2 : end), ...
                             car_1_braking_motion_time_grid(2 : end), ...
                             car_1_left_motion_time_grid(2 : end)] ...
                        )';

                        car_1_total_motion_x_y_vx_vy_phi_omega_grid = [...
                            car_1_total_motion_x_y_vx_vy_phi_omega_grid; ...
                            car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(2 : end, :); ...
                            car_1_braking_motion_x_y_vx_vy_phi_omega_grid(2 : end, :); ...
                            car_1_left_motion_x_y_vx_vy_phi_omega_grid(2 : end, :)...
                        ];
                        
                            if (~isfield(car_1_left_motion_sol, 'xe')) || ...
                           (numel(car_1_left_motion_sol.xe) == 0)       % 1й автомобиль не врезался
                            
                            car_2_total_motion_x_y_vx_vy_phi_omega_grid = [...
                                car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                                car_2_x_y_vx_vy_phi_omega_grid(2 : end, :)
                            ];
                            else
                            
                            happened_road_accident = true;
                            
                            car_2_total_motion_x_y_vx_vy_phi_omega_grid = [...
                                car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                                car_2_x_y_vx_vy_phi_omega_grid(2 : ...
                                    numel(car_1_reaction_motion_time_grid(1 : end)) + ...
                                    numel(car_1_braking_motion_time_grid(2 : end)) + ...
                                    numel(car_1_left_motion_time_grid(2 : end)), :)
                            ];
                            end
    
                        disp('FINISH 1st car left motion modelling till t_end');
                        else
                        
                        happened_road_accident = true;
                        disp('road accident happened');
            
                        total_motion_time_grid = (...
                            [total_motion_time_grid, ...
                             car_1_reaction_motion_time_grid(2 : end), ...
                             car_1_braking_motion_time_grid(2 : end)] ...
                        )';

                        car_1_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                            car_1_total_motion_x_y_vx_vy_phi_omega_grid; ...
                            car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(2 : end, :); ...
                            car_1_braking_motion_x_y_vx_vy_phi_omega_grid(2 : end, :) ...
                        ];
                                
                        car_2_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                            car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                            car_2_x_y_vx_vy_phi_omega_grid(2 : ...
                                (numel(car_1_reaction_motion_time_grid) + numel(car_1_braking_motion_time_grid) - 1), ...
                                :) ...
                        ]; 
                        end
                    
                    opts = rmfield(opts, event_names.intersection_opts);
                    
                    else
                    
                    % время моделирования истекло или произошло столкновение за время
                    % реакции 1го автомобиля
        
                    disp('Время истекло или столкновение за время реакции');
        
                    happened_road_accident = true;
        
                    total_motion_time_grid = (...
                        [total_motion_time_grid, ...
                        car_1_reaction_motion_time_grid(2 : end)]...
                    )';
        
                    car_1_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                        car_1_total_motion_x_y_vx_vy_phi_omega_grid; ...
                        car_1_reaction_motion_x_y_vx_vy_phi_omega_grid(2 : end, :)...
                    ];
                                       
                    car_2_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                        car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                        car_2_x_y_vx_vy_phi_omega_grid(2 : numel(car_1_reaction_motion_time_grid), :) ...
                    ];
                
                    opts = rmfield(opts, event_names.intersection_opts);
                   end
                    
        else
            
            % манёвр не требуется, продолжаем движение с той же скоростью
                   
            disp('START 1st car left motion modelling till t_end');
                   
            car_1_left_motion_time_grid = car_2_time_grid;
            car_1_v = norm([car_1_vx, car_1_vy]);

            car_1_left_motion_param = ...
                get_car_straight_motion_param_struct(car_1_x, car_1_y, ...
                                                     car_1_v, car_1_phi, ...
                                                     car_1_acc_tan, ...
                                                     car_1_left_motion_time_grid);
            
            opts = rmfield(opts, event_names.time_limit_opts);
                   
            car_1_left_motion_sol = move_forward(car_1_left_motion_param, opts);
                        
            car_1_left_motion_time_grid = car_1_left_motion_sol.x;
            car_1_left_motion_x_y_vx_vy_phi_omega_grid = (car_1_left_motion_sol.y)';
            
            disp('Временная сетка: ');
            disp(car_1_left_motion_time_grid');
            disp('Параметры движения 1го автомобиля: ');
            disp(car_1_left_motion_x_y_vx_vy_phi_omega_grid);
                   
            disp('FINISH 1st car left motion modelling till t_end');
                   
            total_motion_time_grid = ([ ...
                total_motion_time_grid, ...
                car_1_left_motion_time_grid(2 : end) ...
            ])';
                   
            car_1_total_motion_x_y_vx_vy_phi_omega_grid = [ ...
                car_1_total_motion_x_y_vx_vy_phi_omega_grid; ...
                car_1_left_motion_x_y_vx_vy_phi_omega_grid(2 : end, :) ...
            ];
               
            car_2_total_motion_x_y_vx_vy_phi_omega_grid = [
                car_2_total_motion_x_y_vx_vy_phi_omega_grid; ...
                car_2_x_y_vx_vy_phi_omega_grid(2 : end, :)
            ];
               
            acceptable_v_dist_curv = [acceptable_v_dist_curv; ...
                                               repmat(car_1_v, car_1_curvature_grid_len, 1), ...
                                                 repmat(car_2_y - car_1_y, car_1_curvature_grid_len, 1), ...
                                                 car_1_curvature_grid'];
                    
        end
                    
            end 
         
        end
        
% Отрисовка
% 
%         disp('START drawing');
% 
%         ax.Title.String = ['Modelling: car\_1\_v = ', num2str(car_1_v_start), ' м/с,   cars\_distance = ', num2str(car_2_y_start - car_1_start_motion_param.y)];
%         
%         total_motion_time_grid_len = numel(total_motion_time_grid); 
%         total_motion_time_grid_indices = (1 : 1 : numel(total_motion_time_grid))';
% 
%         size(total_motion_time_grid)
%         size(car_1_total_motion_x_y_vx_vy_phi_omega_grid)
%         size(car_2_total_motion_x_y_vx_vy_phi_omega_grid)
%         
%         print_results( ... 
%             [ total_motion_time_grid_indices, total_motion_time_grid ], ...
%             [ total_motion_time_grid_indices, car_1_total_motion_x_y_vx_vy_phi_omega_grid ], ...
%             [ total_motion_time_grid_indices, car_2_total_motion_x_y_vx_vy_phi_omega_grid ] ...
%         );
% 
%         % video_writer = VideoWriter('tmp.avi'); 
%         % video_writer.FrameRate = 10;
%                     
%         draw_motion(fig, ...
%                     total_motion_time_grid, ...
%                     car_1_total_motion_x_y_vx_vy_phi_omega_grid, ...
%                     car_2_total_motion_x_y_vx_vy_phi_omega_grid, ...
%                     happened_road_accident, ...
%                     car_dimensions, ...
%                     road);
%         
%         disp('FINISH drawing');

          disp(['FINISH modelling: car_1_v = ', num2str(car_1_v_start), ...
              ' м/с, cars_distance = ', num2str(car_2_y_start - car_1_start_motion_param.y), ... 
              ' м, road_accident_happened: ', num2str(happened_road_accident)]);
    
    end
    
end

disp('FINISH program');