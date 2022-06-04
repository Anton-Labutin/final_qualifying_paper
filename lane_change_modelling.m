% Моделирование перестроения автомобиля на левую и правую полосу

clear;
clc;

%% Входные данные

    % общие параметры

[field_length, field_width] = get_field_size();
t_start = 0;
t_point_cnt = 20;

eps = 1e-1;

    % параметры дороги

[left_side_pos, lane_width, lane_cnt] = get_road_param(field_width);

    % габариты автомобиля

[length, width, mass] = get_car_param();

    % параметры дивжения     

x_0 = left_side_pos + 1.5 * lane_width;
y_0 = field_length / 10;
v_0 = 5;
phi_0 = 0;

rot_angle = pi / 6;
acc_rot_1 = 0;
R_1 = lane_width / (3 * (1 + cos(pi - rot_angle))); % по оси х переместиться на lane_width / 3, повернувшиь на угол rot_angle
t_end_1 = rot_angle * R_1 / v_0;

t_end_2 = lane_width / (3 * v_0 * sin(rot_angle));
acc_straight_2 = 0;

acc_rot_3 = 0; 
R_3 = lane_width / (3 * (1 - cos(rot_angle))); 
t_end_3 = R_3 * rot_angle / v_0;

lane_change_right = false;

field_names = get_field_names();

t_end = t_start + t_end_1 + t_end_2 + t_end_3 + 0.1;

opts.ode_opts = odeset('Refine', 2);
opts.time_limit_opts = struct(field_names.t, t_end, field_names.eps, eps);

disp('Input parameters read');

%% Перестроение

t_grid_1 = linspace(t_start, t_end_1, t_point_cnt);
t_grid_2 = linspace(t_grid_1(end), t_grid_1(end) + t_end_2, t_point_cnt);
t_grid_3 = linspace(t_grid_2(end), t_grid_2(end) + t_end_3, t_point_cnt);

turn_1_motion_param = struct(field_names.x, x_0, ...
                             field_names.y, y_0, ...
                             field_names.v, v_0, ...
                             field_names.phi, phi_0, ...
                             field_names.acc_rot, acc_rot_1, ...
                             field_names.t, t_grid_1, ...
                             field_names.R, R_1);
                
straight_motion_param = struct(field_names.t, t_grid_2, ...
                               field_names.acc_tan, acc_straight_2);
                  
turn_2_motion_param = struct(field_names.t, t_grid_3, ...
                             field_names.R, R_3, ...
                             field_names.acc_rot, acc_rot_3);
                         

[t_grid, x_y_vx_vy_phi_omega_grid, te, ye, ie] = lane_change(turn_1_motion_param, straight_motion_param, turn_2_motion_param, opts, lane_change_right);

if ~lane_change_right
    disp('Lane change to the left completed');
else
    disp('Lane change to the right completed');
end

%% Отрисовка 

figure('Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Position', [0 0 1 1]);

t_grid
x_y_vx_vy_phi_omega_grid

for i = 1 : numel(t_grid)
    frame = get_car_frame(x_y_vx_vy_phi_omega_grid(i, 1), ...
                           x_y_vx_vy_phi_omega_grid(i, 2), ...
                           x_y_vx_vy_phi_omega_grid(i, 5), ...
                           width, length);
    
    frame = [frame(1, :), frame(1, 1); ...
             frame(2, :), frame(2, 1)];
    
%   plot(x_y_vx_vy_phi_omega_grid_total(i, 1), x_y_vx_vy_phi_omega_grid_total(i, 2), '*');
    plot(frame(1, :), frame(2, :), 'b', ...
         frame(1, 1), frame(2, 1), 'r*', ...
         x_y_vx_vy_phi_omega_grid(i, 1), x_y_vx_vy_phi_omega_grid(i, 2), 'g*');

    % plot(frame);

    for i = 0 : lane_cnt
        xline(left_side_pos + i * lane_width);
    end

    xlim([0, field_width]);
    ylim([0, field_length]);
    
    ax = gca;
    ax.DataAspectRatio = [1, 1, 1];
    set(gca, 'XTick', [], 'YTick', []);
    
    if i < numel(t_grid)
         pause(t_grid(i + 1) - t_grid(i));
    end
end