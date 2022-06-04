function [d_x_y_vx_vy_phi_omega] = ...
    two_cars_combined_motion_eq(t, x_y_vx_vy_phi_omega, car_1_motion_param, car_2_motion_param)

% v_straight_0 - скорость прямолинейного движения
% sign_acc_straight - ускорение при прямолинейном дивжении: > 0 - равноускоренное
%      движение, < 0 - равнозамедленное
% sign_acc_rot - угловое ускорение: > 0 - движение по часовой стрелке, < 0
%      - против
% R - радиус окружности 
% phi_straight_0 - угол между начальным вектором скорости и осью OY при
%       прямолинейном движении

% параметры движения 1го автомобиля 

v0_straight_1 = car_1_motion_param.v0_straight;
sign_acc_straight_1 = car_1_motion_param.sign_acc_straight;
phi_straight_1 = car_1_motion_param.phi_straight;
sign_acc_rot_1 = car_1_motion_param.sign_acc_rot;
R_1 = car_1_motion_param.R;

phi_1 = x_y_vx_vy_phi_omega(5);
omega_1 = x_y_vx_vy_phi_omega(6);

v_straight_1 = v0_straight_1 + sign_acc_straight_1 * t;
v_linear_1 = omega_1 * R_1;

% параметры движения 2го автомобиля

v0_straight_2 = car_2_motion_param.v0_straight;
sign_acc_straight_2 = car_2_motion_param.sign_acc_straight;
phi_straight_2 = car_2_motion_param.phi_straight;
sign_acc_rot_2 = car_2_motion_param.sign_acc_rot;
R_2 = car_2_motion_param.R;
         
phi_2 = x_y_vx_vy_phi_omega(11);
omega_2 = x_y_vx_vy_phi_omega(12);

v_straight_2 = v0_straight_2 + sign_acc_straight_2 * t;
v_linear_2 = omega_2 * R_2;

% уравнения движения 1го автомобиля

d_x_y_vx_vy_phi_omega(1) = v_straight_1 * sin(phi_straight_1) + v_linear_1 * sin(phi_1);                                                   % координата x
d_x_y_vx_vy_phi_omega(2) = v_straight_1 * cos(phi_straight_1) + v_linear_1 * cos(phi_1);                                                   % координата y
d_x_y_vx_vy_phi_omega(3) = sign_acc_straight_1 * sin(phi_straight_1) + R_1 * (sign_acc_rot_1 * sin(phi_1) + omega_1^2 * cos(phi_1));       % скорость vx
d_x_y_vx_vy_phi_omega(4) = sign_acc_straight_1 * cos(phi_straight_1) + R_1 * (sign_acc_rot_1 * cos(phi_1) - omega_1^2 * sin(phi_1));       % скорость vy
d_x_y_vx_vy_phi_omega(5) = omega_1;                                                                                                        % угол от pi (по часовой стрелке > 0)
d_x_y_vx_vy_phi_omega(6) = sign_acc_rot_1;                                                                                                 % угловая скорость

% уравнения движения 2го автомобиля 

d_x_y_vx_vy_phi_omega(7) = v_straight_2 * sin(phi_straight_2) + v_linear_2 * sin(phi_2);                                                   % координата x
d_x_y_vx_vy_phi_omega(8) = v_straight_2 * cos(phi_straight_2) + v_linear_2 * cos(phi_2);                                                   % координата y
d_x_y_vx_vy_phi_omega(9) = sign_acc_straight_2 * sin(phi_straight_2) + R_2 * (sign_acc_rot_2 * sin(phi_2) + omega_2^2 * cos(phi_2));       % скорость vx
d_x_y_vx_vy_phi_omega(10) = sign_acc_straight_2 * cos(phi_straight_2) + R_2 * (sign_acc_rot_2 * cos(phi_2) - omega_2^2 * sin(phi_2));      % скорость vy
d_x_y_vx_vy_phi_omega(11) = omega_2;                                                                                                       % угол от pi (по часовой стрелке > 0)
d_x_y_vx_vy_phi_omega(12) = sign_acc_rot_2;                                                                                                % угловая скорость 

end