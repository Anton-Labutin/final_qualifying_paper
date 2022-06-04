% Движение автомобиля по окружности
clear;
clc;

%% входные данные

    % общие параметры

[field_length, field_width] = get_field_size();

t_end = 12; % время моделирования в сек

    % параметры дороги

[left_side_pos, lane_width, lane_cnt] = get_road_param(field_width);

    % габариты автомобиля

[length, width, mass] = get_car_param();

    % начальная позиция автомобиля
   
x_0 = 0.5 * field_width; 
y_0 = 0.5 * field_length; 

phi_0 = pi; % угол поворота машины от оси OY (рад)

v_0 = 5; % начальная скорость автомобиля (м/с) 

R = 10; % радиус кривизны кривой


%% вычисления 

    % движение по окружности с постоянной скоростью 

opts = odeset('Refine', 24);

[t_grid, x_y_vx_vy_phi_omega_grid] = ode45(@(t, x_y_vx_vy_phi_omega) combined_motion_eq(t, x_y_vx_vy_phi_omega, 0, 0, 0, R, 0), ...
    [0, t_end], ...
    [x_0, y_0, v_0 * sin(phi_0), v_0 * cos(phi_0), phi_0, -v_0 / R], ...
    opts)


%% графики

figure('Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Position', [0 0 1 1]);
hold on; %% входные данные


for i = 1 : numel(t_grid)
    [x, y] = get_car_frame(x_y_vx_vy_phi_omega_grid(i, 1), x_y_vx_vy_phi_omega_grid(i, 2), x_y_vx_vy_phi_omega_grid(i, 5), width, length);
    
    x = [x, x(1)];
    y = [y, y(1)];
    
    % plot(x_y_vx_vy_phi_grid(i, 1), x_y_vx_vy_phi_grid(i, 2), '*');
    plot(x, y, 'b', x(1), y(1), 'r*', x_y_vx_vy_phi_omega_grid(i, 1), x_y_vx_vy_phi_omega_grid(i, 2), 'g*');
    
    xlim([0, field_width]);
    ylim([0, field_length]);
    
    ax = gca;
    ax.DataAspectRatio = [1, 1, 1];
    set(gca, 'XTick', [], 'YTick', []);
    
    if i < numel(t_grid)
        pause(t_grid(i + 1) - t_grid(i));
    end
end