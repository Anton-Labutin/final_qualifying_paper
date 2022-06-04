% Моделирование торможения управляемого автомобиля при перестроении 2го
% автомобиля

clear;
clc;

%% входные данные

    % общие параметры

[field_length, field_width] = get_field_size();

t_start = 0;                                % время начала моделирования (сек)
t_end = 6;                                 % время конца моделирования (сек)
t_change = t_start + (t_end - t_start) / 4; % время начала перестроения 2го автомобиля (сек)
t_reaction = 1.0;                             % время реации автомобиля (сек)
t_scale_factor = 1 / 200;

eps = 1e-3;

    % параметры дороги

[left_side_pos, lane_width, lane_cnt] = get_road_param(field_width);

    % габариты автомобиля

[length, width, mass] = get_car_param();

    % параметры дивжения 
    
        % управляемый автомобиль

x_0_1 = left_side_pos + 1.5 * lane_width;
y_0_1 = field_length / 10;
        
v_0_1 = 15;
acc_tan_1 = 0;
phi_0_1 = 0;

acc_tan_change_1 = -3;

        % второй автомобиль
        
x_0_2 = x_0_1 + lane_width;
y_0_2 = 3 * y_0_1; 

v_0_2 = 10;
acc_tan_2 = 0;
phi_0_2 = 0;

%% Моделирование

disp('START modelling');

field_names = get_field_names();
event_names = get_event_names();

t_step = (t_end - t_start) * t_scale_factor;

opts.ode_opts = odeset('RelTol', eps, ...
                       'AbsTol', eps, ...
                       'InitialStep', t_step, ...
                       'MaxStep', t_step);

% ---- прямолинейное движение обоих автомобилей до начала перестроения 2го автомобиля    

% -------- начальные параметры движения 

% t_grid = linspace(t_start, t_change, t_point_cnt); 
% t_grid = t_start : t_step : t_change;

straight_motion_param_1 = struct(field_names.x, x_0_1, ...
                                 field_names.y, y_0_1, ...
                                 field_names.v, v_0_1, ...
                                 field_names.phi, phi_0_1, ...
                                 field_names.acc_tan, acc_tan_1, ...
                                 field_names.t, [t_start, t_change]);
                                 % field_names.t, t_grid);
    
straight_motion_param_2 = struct(field_names.x, x_0_2, ...
                                 field_names.y, y_0_2, ...
                                 field_names.v, v_0_2, ...
                                 field_names.phi, phi_0_2, ...
                                 field_names.acc_tan, acc_tan_2, ...
                                 field_names.t, [t_start, t_change]);
                                 % field_names.t, t_grid);
                       
                   
opts.time_limit_opts = struct(field_names.t, t_end, ...
                              field_names.eps, eps);

% -------- моделирование прямолинейного движения двух автомобилей до начала
% перестроения 2го автомобиля либо истечения времени моделирования 

disp('START modelling before lane change');

% [t_grid, x_y_vx_vy_phi_omega_grid_1, te, ye, ie] = move_forward(straight_motion_param_1, opts);
% [t_grid, x_y_vx_vy_phi_omega_grid_2, te, ye, ie] = move_forward(straight_motion_param_2, opts);
sol_1 = move_forward(straight_motion_param_1, opts);

straight_motion_param_2 = setfield(straight_motion_param_2, field_names.t, sol_1.x);
opts = rmfield(opts, event_names.time_limit_opts);

sol_2 = move_forward(straight_motion_param_2, opts);

opts.time_limit_opts = struct(field_names.t, t_end, ...
                              field_names.eps, eps);

t_grid = sol_2.x;
x_y_vx_vy_phi_omega_grid_1 = (sol_1.y)';
x_y_vx_vy_phi_omega_grid_2 = (sol_2.y)';

disp('Временная сетка: ');
disp(t_grid');
disp('Параметры движения 1го автомобиля: ');
disp(x_y_vx_vy_phi_omega_grid_1);
disp('Параметры движения 2го автомобиля: ');
disp(x_y_vx_vy_phi_omega_grid_2);

t_grid_total = t_grid;                                                % итоговая временная сетка
x_y_vx_vy_phi_omega_grid_1_total = x_y_vx_vy_phi_omega_grid_1;        % итоговая сетка позиций 1го автомобиля
x_y_vx_vy_phi_omega_grid_2_total = x_y_vx_vy_phi_omega_grid_2; % итоговая сетка позиций 2го автомобиля

disp('FINISH modelling before lane change');

if (~isfield(sol_1, 'xe')) || (numel(sol_1.xe) == 0) % время моделирования не истекло

    disp('modelling not finished yet');
    
% ---- перестроение 2го автомобиля и реакция 1го автомобиля
    
% --------- перестроение 2го автомобиля

% ------------ начальные параметры 2го автомобиля перед перестроением 
            
    [x_0_2, y_0_2, vx_0_2, vy_0_2, phi_0_2, omega_0_2] = ...
        get_cur_pos(x_y_vx_vy_phi_omega_grid_2_total(end, :));
        
% ---------------- параметры для первого поворота при перестроении
                
    rot_angle = pi / 6;

    v_0_2 = norm([vx_0_2, vy_0_2]);
    turn_1_acc_rot_2 = 0;
    turn_1_R_2 = lane_width / (3 * (1 + cos(pi - rot_angle))); % по оси х переместиться на lane_width / 3, повернувшиь на угол rot_angle;

    turn_1_t_2 = rot_angle * turn_1_R_2 / v_0_2;
%     turn_1_t_grid_2 = linspace(t_grid_total(end), ...
%                                t_grid_total(end) + turn_1_t_2, ...
%                                t_point_cnt / 2);

    t_start = t_grid_total(end);
    % t_step_turn_1 = turn_1_t_2 * t_scale_factor * 10;
    % turn_1_t_grid_2 = t_start : t_step_turn_1 : (t_start + turn_1_t_2);

    turn_1_motion_param_2 = struct(field_names.x, x_0_2, ...
                                   field_names.y, y_0_2, ...
                                   field_names.v, v_0_2, ...
                                   field_names.phi, phi_0_2, ...
                                   field_names.acc_rot, turn_1_acc_rot_2, ...
                                   field_names.R, turn_1_R_2, ...
                                   field_names.t, [t_start, t_start + turn_1_t_2]);
                                   % field_names.t, turn_1_t_grid_2, ...
                                   

% ---------------- параметры для прямолинейного движения при перестроении

    straight_motion_acc_tan_2 = 0;

    straight_motion_t_2 = lane_width / (3 * v_0_2 * sin(rot_angle));
%     straight_motion_t_grid_2 = linspace(turn_1_t_grid_2(end), ...
%                                         turn_1_t_grid_2(end) + straight_motion_t_2, ...
%                                         t_point_cnt / 2);

    t_start = t_start + turn_1_t_2;
    % t_start = turn_1_t_grid_2(end);
    % t_step_straight_motion = straight_motion_t_2 * t_scale_factor * 10;
    % straight_motion_t_grid_2 = t_start : t_step_straight_motion : (t_start + straight_motion_t_2);

    straight_motion_param_2 = struct(field_names.acc_tan, straight_motion_acc_tan_2, ...
                                     field_names.t, [t_start, t_start + straight_motion_t_2]);
                                    % field_names.t, straight_motion_t_grid_2)
                                     
                                    

% ---------------- параметры второго поворота при перестроении
                             
    turn_2_acc_rot_2 = 0;
    turn_2_R_2 = lane_width / (3 * (1 - cos(rot_angle)));

    turn_2_t_2 =  turn_2_R_2 * rot_angle / v_0_2;
%     turn_2_t_grid_2 = linspace(straight_motion_t_grid_2(end), ...
%                                straight_motion_t_grid_2(end) + turn_2_t_2, ...
%                                t_point_cnt / 2);

    t_start = t_start + straight_motion_t_2;
    % t_start = straight_motion_t_grid_2(end);
    % t_step_turn_2 = turn_2_t_2 * t_scale_factor * 10;
    % turn_2_t_grid_2 = t_start : t_step_turn_2 : (t_start + turn_2_t_2);
    
    turn_2_motion_param_2 = struct(field_names.acc_rot, turn_2_acc_rot_2, ...
                                   field_names.R, turn_2_R_2, ...
                                   field_names.t, [t_start, t_start + turn_2_t_2]);
                                   % field_names.t, turn_2_t_grid_2);
    
% ------------ моделирование перестроения 2го автомобиля (частичное, если истечёт время моделирования)

%     opts.time_limit_opts = setfield(opts.time_limit_opts, ...
%                                     field_names.t, ...
%                                     t_end - t_grid_total(end));
                                
%     t_step_change = min([t_step_turn_1, t_step_straight_motion, t_step_turn_2]);
%     opts.ode_opts = odeset(opts.ode_opts, 'InitialStep', t_step_change, ...
%                                          'MaxStep', t_step_change);

    disp('START 2d car lane change modelling');
      
    % [t_grid_change_2, x_y_vx_vy_phi_omega_grid_change_2, te, ye, ie] = ...
    sol_change_2 = ...
        lane_change(turn_1_motion_param_2, ...
                    straight_motion_param_2, ...
                    turn_2_motion_param_2, ...
                    opts, ...
                    false);
    t_grid_change_2 = sol_change_2.x;
    x_y_vx_vy_phi_omega_grid_change_2 = (sol_change_2.y)';
                
                
    disp('Временная сетка: ');
    disp(t_grid_change_2');
    disp('Параметры движения 2го автомобиля: ');
    disp(x_y_vx_vy_phi_omega_grid_change_2);
    
    
    t_grid_2 = t_grid_change_2;
    x_y_vx_vy_phi_omega_grid_2 = x_y_vx_vy_phi_omega_grid_change_2;
    
    disp('FINISH 2d car lane change modelling');

    if ~isfield(sol_change_2, 'xe') || numel(sol_change_2.xe) == 0 % время модeлирования не закончилось при перестроении 2го автомобиля

        disp('modelling not finished yet');
% -------- моделирование прямолинейного движения 2го автомобиля до
% конца времени моделирования 

% ------------ параметры движения

        [x_0_2, y_0_2, vx_0_2, vy_0_2, phi_0_2, omega_0_2] = ...
            get_cur_pos(x_y_vx_vy_phi_omega_grid_change_2(end, :));

        % t_grid_left_2 = linspace(t_grid_change_2(end), t_end, 3 * t_point_cnt);
        t_start = t_grid_change_2(end);
        % t_step = (t_end - t_start) * t_scale_factor;
        % t_grid_left_2 = t_start : t_step : t_end;
        
        % t_step_change = min([t_step_change, t_step]);
        
        v_0_2 = norm([vx_0_2, vy_0_2]);
        
        straight_motion_param_2 = struct(field_names.x, x_0_2, ...
                                         field_names.y, y_0_2, ...
                                         field_names.v, v_0_2, ...
                                         field_names.phi, phi_0_2, ...
                                         field_names.acc_tan, acc_tan_2, ...
                                         field_names.t, [t_start, t_end]);
                                         % field_names.t, t_grid_left_2);
        
        opts = rmfield(opts, event_names.time_limit_opts);
        
        % opts.time_limit_opts = setfield(opts.time_limit_opts, ...
        %                                field_names.t, ...
        %                                t_end - t_start);
                                    
        
                                    
%         opts.ode_opts = odeset(opts.ode_opts, 'InitialStep', t_step, ...
%                                               'MaxStep', t_step);

% ------------ моделирование движения до конца времени моделирования

        disp('START 2d car left motion modelling till t_end');

        % [t_grid_left_2, x_y_vx_vy_phi_omega_grid_left_2] = ...
        sol_left_2 = ...
            move_forward(straight_motion_param_2, opts);
        t_grid_left_2 = sol_left_2.x;
        x_y_vx_vy_phi_omega_grid_left_2 = (sol_left_2.y)';
            
        disp('Временная сетка: ');
        disp(t_grid_left_2');
        disp('Параметры движения 2го автомобиля: ');
        disp(x_y_vx_vy_phi_omega_grid_left_2);
   
        t_grid_2 = [t_grid_2, t_grid_left_2(2 : end)];
        x_y_vx_vy_phi_omega_grid_2 = [x_y_vx_vy_phi_omega_grid_2; ...
                                      x_y_vx_vy_phi_omega_grid_left_2(2 : end, :)];
    
        disp('FINISH 2d car left motion modelling till t_end');
    end

% -------- движение 1го автомобиля

happened_road_accident = false;

% ------------ движение за время реакции

% ---------------- начальные параметры движения

    [x_0_1, y_0_1, vx_0_1, vy_0_1, phi_0_1, omega_0_1] = ...
        get_cur_pos(x_y_vx_vy_phi_omega_grid_1_total(end, :));

    t_reaction_mask = (t_grid_2 <= (t_grid_2(1) + t_reaction));
    t_grid_reaction = t_grid_2(t_reaction_mask);
    
    v_0_1 = norm([vx_0_1, vy_0_1]);

    reaction_motion_param_1 = struct(field_names.x, x_0_1, ...
                                     field_names.y, y_0_1, ...
                                     field_names.v, v_0_1, ...
                                     field_names.phi, phi_0_1, ...
                                     field_names.acc_tan, acc_tan_1, ...
                                     field_names.t, t_grid_reaction);

    % параметры для отслеживания столкновения за время реакции 1го
    % автомобиля
    opts.intersection_opts = struct(field_names.x, x_y_vx_vy_phi_omega_grid_2(1 : numel(t_grid_reaction), 1), ...
                                    field_names.y, x_y_vx_vy_phi_omega_grid_2(1 : numel(t_grid_reaction), 2), ...
                                    field_names.phi, x_y_vx_vy_phi_omega_grid_2(1 : numel(t_grid_reaction), 5), ...
                                    field_names.length, length, ...
                                    field_names.width, width, ...
                                    field_names.t, t_grid_reaction);
                       
    % время моделирования не учитываем, т.к. оно учтено уже в t_grid_2
    % opts = rmfield(opts, event_names.time_limit_opts);
    
%     opts.ode_opts = odeset(opts.ode_opts, 'InitialStep', t_step_change, ...
%                                           'MaxStep', t_step_change);

% ---------------- моделирование движения за время реакции (либо меньше, если время моделирования истекло или произошло столкновение)

    disp('START 1st car reaction modelling');

    % [t_grid_reaction, x_y_vx_vy_phi_omega_grid_reaction_1, te, ye, ie] = move_forward(reaction_motion_param_1, opts);
    sol_reaction_1 = move_forward(reaction_motion_param_1, opts);
    t_grid_reaction = sol_reaction_1.x;
    x_y_vx_vy_phi_omega_grid_reaction_1 = (sol_reaction_1.y)';
    
    disp('Временная сетка: ');
    disp(t_grid_reaction');
    disp('Параметры движения 1го автомобиля: ');
    disp(x_y_vx_vy_phi_omega_grid_reaction_1);
    
    disp('FINISH 1st car reaction modelling');
    
    if (~isfield(sol_reaction_1, 'xe')) || (numel(sol_reaction_1.xe) == 0) % время моделирования не истекло и не произошло столкновение

        disp('modelling not finished yet');
% ------------ торможение 

% ---------------- параметры торможения

        [x_0_1, y_0_1, vx_0_1, vy_0_1, phi_0_1, omega_0_1] = ...
            get_cur_pos(x_y_vx_vy_phi_omega_grid_reaction_1(end, :));


        t_grid_braking = [t_grid_reaction(end), ...
            t_grid_2(~t_reaction_mask)];

        v_0_1 = norm([vx_0_1, vy_0_1]);

        braking_motion_param_1 = struct(field_names.x, x_0_1, ...
                                 field_names.y, y_0_1, ...
                                 field_names.v, v_0_1, ...
                                 field_names.phi, phi_0_1, ...
                                 field_names.acc_tan, acc_tan_change_1, ...
                                 field_names.t, t_grid_braking);
                             
        % обновляем параметры для отслеживания столкновения во время
        % торможения 1го автомобиля
        opts.intersection_opts = setfield(opts.intersection_opts, ...
                                          field_names.x, ...
                                          x_y_vx_vy_phi_omega_grid_2(numel(t_grid_reaction) : end, 1));
                                      
        opts.intersection_opts = setfield(opts.intersection_opts, ...
                                          field_names.y, ...
                                          x_y_vx_vy_phi_omega_grid_2(numel(t_grid_reaction) : end, 2));
        
        opts.intersection_opts = setfield(opts.intersection_opts, ...
                                          field_names.phi, ...
                                          x_y_vx_vy_phi_omega_grid_2(numel(t_grid_reaction) : end, 5));
                                      
        opts.intersection_opts = setfield(opts.intersection_opts, ...
                                          field_names.t, ...
                                          t_grid_braking);
        
        % параметры для отслеживания момента достижения конечной скорости
        opts.motion_opts = struct(field_names.v, 5, ...
                                  field_names.eps, eps); 


% ---------------- моделирование торможения до наступления хотя бы одного
% из событий: 
% 1. Время моделирования закончилось (учтено в t_grid_2)
% 2. Скорость машины стала = 0
% 3. Произошло столкновение машин

        disp('START 1st car braking modelling');

        % [t_grid_braking, x_y_vx_vy_phi_omega_grid_braking_1, te, ye, ie] = ...
        sol_braking_1 = ...
            move_forward(braking_motion_param_1, opts);    
        t_grid_braking = sol_braking_1.x;
        x_y_vx_vy_phi_omega_grid_braking_1 = (sol_braking_1.y)';
            
        disp('Временная сетка: ');
        disp(t_grid_braking');
        disp('Параметры движения 1го автомобиля: ');
        disp(x_y_vx_vy_phi_omega_grid_braking_1);
        
        disp('FINISH 1st car braking modelling');
     
 
        if (~isfield(sol_braking_1, 'xe')) || (numel(sol_braking_1.xe) == 0) || (sol_braking_1.ie(1) == 2) % 1й автомобиль остановился и не врезался
            % ------------ моделирование движения 1го автомобиля до конца времени
            % моделирования
            
            disp('road accident not happened');
            
            [x_0_1, y_0_1, vx_0_1, vy_0_1, phi_0_1, omega_0_1] = ...
                get_cur_pos(x_y_vx_vy_phi_omega_grid_braking_1(end, :));

            t_grid_left_1_mask = (t_grid_2 >= t_grid_braking(end));
            t_grid_left_1 = t_grid_2(t_grid_left_1_mask);
    
            v_0_1 = norm([vx_0_1, vy_0_1]);

            left_motion_param_1 = struct(field_names.x, x_0_1, ...
                                         field_names.y, y_0_1, ...
                                         field_names.v, v_0_1, ...
                                         field_names.phi, phi_0_1, ...
                                         field_names.acc_tan, acc_tan_1, ...
                                         field_names.t, t_grid_left_1);
                                     
            opts = rmfield(opts, event_names.intersection_opts);
            opts = rmfield(opts, event_names.motion_opts);
        
            disp('START 1st car left motion modelling till t_end');

            sol_left_1 = ...
                move_forward(left_motion_param_1, opts);
            t_grid_left_1 = sol_left_1.x;
            x_y_vx_vy_phi_omega_grid_left_1 = (sol_left_1.y)';
            
            disp('Временная сетка: ');
            disp(t_grid_left_1);
            disp('Параметры движения 1го автомобиля: ');
            disp(x_y_vx_vy_phi_omega_grid_left_1);
   
            t_grid_total = ([t_grid_total, ...
                             t_grid_reaction(2 : end), ...
                             t_grid_braking(2 : end), ...
                             t_grid_left_1(2 : end)])';

            x_y_vx_vy_phi_omega_grid_1_total = [x_y_vx_vy_phi_omega_grid_1_total; ...
                                                x_y_vx_vy_phi_omega_grid_reaction_1(2 : end, :); ...
                                                x_y_vx_vy_phi_omega_grid_braking_1(2 : end, :); ...
                                                x_y_vx_vy_phi_omega_grid_left_1(2 : end, :)];
                                
            x_y_vx_vy_phi_omega_grid_2_total = [x_y_vx_vy_phi_omega_grid_2_total; ...
                                                x_y_vx_vy_phi_omega_grid_2(2 : end, :)]; 
                                            
            size(x_y_vx_vy_phi_omega_grid_1_total)
            size(x_y_vx_vy_phi_omega_grid_2_total)
            size(t_grid_total)
    
            disp('FINISH 1st car left motion modelling till t_end');
        else 
            happened_road_accident = true;
            disp('road accident happened');
            
            t_grid_total = ([t_grid_total, ...
                             t_grid_reaction(2 : end), ...
                             t_grid_braking(2 : end)])';

            x_y_vx_vy_phi_omega_grid_1_total = [x_y_vx_vy_phi_omega_grid_1_total; ...
                                            x_y_vx_vy_phi_omega_grid_reaction_1(2 : end, :); ...
                                            x_y_vx_vy_phi_omega_grid_braking_1(2 : end, :)];
                                
            x_y_vx_vy_phi_omega_grid_2_total = [x_y_vx_vy_phi_omega_grid_2_total; ...
                                            x_y_vx_vy_phi_omega_grid_2(2 : ...
                                                (numel(t_grid_reaction) + numel(t_grid_braking) - 1), :)]; 
        end
        
  
                                            
        
    else 
        % время моделирования истекло или произошло столкновение за время
        % реакции 1го автомобиля
        
        disp('Время истекло или столкновение за время реакции');
        
        happened_road_accident = true;
        
        t_grid_total = ([t_grid_total, t_grid_reaction(2 : end)])';
        
        x_y_vx_vy_phi_omega_grid_1_total = [x_y_vx_vy_phi_omega_grid_1_total; ...
                                            x_y_vx_vy_phi_omega_grid_reaction_1(2 : end, :)];
                                       
        x_y_vx_vy_phi_omega_grid_2_total = [x_y_vx_vy_phi_omega_grid_2_total; ...
                                            x_y_vx_vy_phi_omega_grid_2(2 : numel(t_grid_reaction), :)];
    end
    
end


                                
%% Отрисовка

% fig_1 = figure;
fig = figure('Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Position', [0 0 1 1]);

[(1 : 1 : numel(t_grid_total))', t_grid_total]
[(1 : 1 : numel(t_grid_total))', x_y_vx_vy_phi_omega_grid_1_total]
[(1 : 1 : numel(t_grid_total))', x_y_vx_vy_phi_omega_grid_2_total]

size(t_grid_total)
size(x_y_vx_vy_phi_omega_grid_1_total)
size(x_y_vx_vy_phi_omega_grid_2_total)
    
t_point_cnt_total = numel(t_grid_total);

% v = VideoWriter('tmp.avi'); % 'Uncompressed AVI'
% v.FrameRate = 10;
% open(v);

for i = 1 : t_point_cnt_total
    frame_1 = get_car_frame(x_y_vx_vy_phi_omega_grid_1_total(i, 1), x_y_vx_vy_phi_omega_grid_1_total(i, 2), x_y_vx_vy_phi_omega_grid_1_total(i, 5), width, length);
    frame_2 = get_car_frame(x_y_vx_vy_phi_omega_grid_2_total(i, 1), x_y_vx_vy_phi_omega_grid_2_total(i, 2), x_y_vx_vy_phi_omega_grid_2_total(i, 5), width, length);
    
    frame_1 = [frame_1(1, :), frame_1(1, 1); 
               frame_1(2, :), frame_1(2, 1)];
    
    frame_2 = [frame_2(1, :), frame_2(1, 1); 
               frame_2(2, :), frame_2(2, 1)];
    
    if i < t_point_cnt_total
        color = 'b';
        color_dot = 'b*';
    else
        if happened_road_accident
            color = 'r';
            color_dot = 'r*';
        end
    end
    
    plot(frame_1(1, :), frame_1(2, :), color, ...
        frame_1(1, 1), frame_1(2, 1), color_dot, ...
        x_y_vx_vy_phi_omega_grid_1_total(i, 1), x_y_vx_vy_phi_omega_grid_1_total(i, 2), color_dot, ...
        frame_2(1, :), frame_2(2, :), color, ...
        frame_2(1, 1), frame_2(2, 1), color_dot, ...
        x_y_vx_vy_phi_omega_grid_2_total(i, 1), x_y_vx_vy_phi_omega_grid_2_total(i, 2), color_dot); 
    
    str = [{['t = ', num2str(t_grid_total(i)), ' сек']}, ...
        {['v_1 = ', num2str(norm([x_y_vx_vy_phi_omega_grid_1_total(i, 3), x_y_vx_vy_phi_omega_grid_1_total(i, 4)])), ' м/с']}, ...
        {['v_2 = ', num2str(norm([x_y_vx_vy_phi_omega_grid_2_total(i, 3), x_y_vx_vy_phi_omega_grid_2_total(i, 4)])), ' м/с']}];
    text(0.8 * field_width, 0.9 * field_length, str, 'FontSize', 14);
    
    for i = 1 : lane_cnt + 1
        xline(left_side_pos + (i - 1) * lane_width);
    end
    
    xlim([0, field_width]);
    ylim([0, field_length]);
    
    ax = gca;
    ax.DataAspectRatio = [1, 1, 1];
    set(gca, 'XTick', [], 'YTick', []);
    
    % writeVideo(v, getframe(gcf));
    
    if i < numel(t_grid_total)
        pause(t_grid_total(i + 1) - t_grid_total(i));
    end
end

% writeVideo(v, getframe(gcf));
% writeVideo(v, getframe(gcf));
% writeVideo(v, getframe(gcf)); 
% writeVideo(v, getframe(gcf));
% writeVideo(v, getframe(gcf));
% 
% close(v);