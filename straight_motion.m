% Движение автомобиля вдоль одной полосы

clear;
clc;

%% входные данные

    % общие параметры

[field_length, field_width] = get_field_size();

t_end = 1; % время моделирования в сек

    % параметры дороги

[left_side_pos, lane_width] = get_lane_param(field_width);

    % габариты автомобиля

[length, width, mass] = get_car_param();

    % начальные параметры движения автомобиля
   
x_0 = left_side_pos + 0.5 * lane_width; % геометрический центр (м)
y_0 = field_length / 10; 

phi_0 = 0; % угол поворота машины от оси OY (рад)

v_0 = 30; % скорость автомобиля (м/с) 

tan_acc_with_sign = -5; % тангенциальное ускорение автомобиля (м/с^2): > 0 - равноускоренное движение, < 0 - равнозамедленное движение

%% вычисления

opts = odeset('Refine', 24, 'NonNegative', [1, 2]);

    % равномерное движение

[t_grid, x_y_vx_vy_phi_grid] = ode45(@(t, xy) straight_motion_eq(t, xy, v_0, 0), ...
    [0, t_end], ...
    [x_0, y_0, v_0 * sin(phi_0), v_0 * cos(phi_0), phi_0], ...
    opts);
    
    % равноускоренное движение
%     
% [t_grid, x_y_vx_vy_phi_grid] = ode45(@(t, xy) straight_motion_eq(t, xy, v_0, tan_acc_with_sign), ...
%     [0, t_end], ...
%     [x_0, y_0, v_0 * sin(phi_0), v_0 * cos(phi_0), phi_0], ...
%     opts);



%% графики

figure('Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Position', [0 0 1 1])

right_side_pos = left_side_pos + lane_width;

for i = 1 : numel(t_grid)
    [x, y] = get_car_frame(x_y_vx_vy_phi_grid(i, 1), x_y_vx_vy_phi_grid(i, 2), x_y_vx_vy_phi_grid(i, 5), width, length);
    x = [x, x(1)]; 
    y = [y, y(1)];
    
    plot(x, y);
    
    % xline(left_side_pos);
    % xline(right_side_pos);
    
    xlim([0, field_width]);
    ylim([0, field_length]);
    
    ax = gca;
    ax.DataAspectRatio = [1, 1, 1];
    set(gca, 'XTick', [], 'YTick', []);
    
    if i < numel(t_grid)
        pause(t_grid(i + 1) - t_grid(i));
    end
end