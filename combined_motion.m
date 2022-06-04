% Комбинированное движение автомобиля

clear;
clc;

%% входные данные



    % параметры движения автомобиля
   
% вариант 1  
    
t_n_count = 4; % количество точек на временной сетке, когда меняются параметры движения
t_n_grid = linspace(0, t_end, t_n_count + 1)    % общие параметры

[field_length, field_width] = get_field_size();

t_end = 20; % время моделирования в сек

    % параметры дороги

% [left_side_pos, lane_width] = get_lane_param(field_width);

    % габариты автомобиля

[length, width, mass] = get_car_param();

x_0 = 0.25 * field_width; 
y_0 = 0.25 * field_length; 

phi_0_grid = zeros(1, numel(t_n_grid)); % угол поворота машины от оси OY (рад)
phi_0_grid(1) = 0;

v_straight_grid = zeros(1, numel(t_n_grid)); % скорость автомобиля (м/с) 
v_straight_grid(1) = 5;
v_straight_grid(2) = 0;
v_straight_grid(3) = 8;
v_straight_grid(4) = 0;

sign_acc_tan_grid = zeros(1, numel(t_n_grid)); % тангенциальное ускорение
sign_acc_tan_grid(1) = 2;
sign_acc_tan_grid(2) = 0;
sign_acc_tan_grid(3) = -2; 
sign_acc_tan_grid(4) = 0;

sign_omega_grid = zeros(1, numel(t_n_grid)); % угловая скорость
sign_omega_grid(1) = 0;
sign_omega_grid(2) = 0.5;
sign_omega_grid(3) = 0;
sign_omega_grid(4) = 0.25;


R_grid = zeros(1, numel(t_n_grid)); % радиус кривизны кривой (м)
R_grid(1) = 0;
R_grid(2) = 10;
R_grid(3) = 0;
R_grid(4) = 20;

%% вычисления 

opts = odeset('MaxStep', 1e-1);

t_grid_total = [];
x_y_vx_vy_phi_grid_total = [];

for i = 1 : t_n_count
    if i > 1
        x_start = x_y_vx_vy_phi_grid(end, 1);
        y_start = x_y_vx_vy_phi_grid(end, 2);
        
        vx_start = x_y_vx_vy_phi_grid(end, 3);
        vy_start = x_y_vx_vy_phi_grid(end, 4);
    else
        x_start = x_0;
        y_start = y_0;
        
        vx_start = v_straight_grid(1) * sin(phi_0_grid(1));
        vy_start = v_straight_grid(1) * cos(phi_0_grid(1));
    end
    
%      [t_grid, x_y_vx_vy_phi_grid] = analyze_motion(v_grid(i), sign_acc_tan_grid(i), ...
%                                      sign_omega_grid(i), R_grid(i));
    
    x_start
    y_start
    vx_start
    vy_start
    phi_0_grid(i)
    v_straight_grid(i)
    sign_acc_tan_grid(i)
    sign_omega_grid(i)
    R_grid(i)
    
    [t_grid, x_y_vx_vy_phi_grid] = ode45(@(t, pos) combined_motion_eq(t, pos, ... 
                                        v_straight_grid(i), sign_acc_tan_grid(i), ...
                                        sign_omega_grid(i), R_grid(i), phi_0_grid(i)), ...
                                        [0, t_n_grid(i + 1) - t_n_grid(i)], ...
                                        [x_start, y_start, vx_start, vy_start, phi_0_grid(i)], ...
                                        opts)
                                    
    t_grid_total = [t_grid_total; t_grid + t_n_grid(i)];
    x_y_vx_vy_phi_grid_total = [x_y_vx_vy_phi_grid_total; x_y_vx_vy_phi_grid]; 
    
    phi_0_grid(i + 1) = x_y_vx_vy_phi_grid(end, 5);
    % v_straight_grid(i + 1) = sqrt(x_y_vx_vy_phi_grid(end, 3)^2 + x_y_vx_vy_phi_grid(end, 4)^2);
end


%% графики

figure('Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Position', [0 0 1 1]);
t_grid_total
x_y_vx_vy_phi_grid_total

for i = 1 : numel(t_grid_total)
    [x, y] = get_car_frame(x_y_vx_vy_phi_grid_total(i, 1), x_y_vx_vy_phi_grid_total(i, 2), x_y_vx_vy_phi_grid_total(i, 5), width, length);
    
    x = [x, x(1)];
    y = [y, y(1)];
    
    % plot(x_y_vx_vy_phi_grid(i, 1), x_y_vx_vy_phi_grid(i, 2), '*');
    plot(x, y, 'b', x(1), y(1), 'r*', x_y_vx_vy_phi_grid_total(i, 1), x_y_vx_vy_phi_grid_total(i, 2), 'g*');
    
    xlim([0, field_width]);
    ylim([0, field_length]);
    
    ax = gca;
    ax.DataAspectRatio = [1, 1, 1];
    set(gca, 'XTick', [], 'YTick', []);
    
    if i < numel(t_grid_total)
        pause(t_grid_total(i + 1) - t_grid_total(i));
    end
end