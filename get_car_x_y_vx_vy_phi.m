function [x, y, vx, vy, phi] = get_car_x_y_vx_vy_phi(x_y_vx_vy_phi)

    x = x_y_vx_vy_phi(1);
    y = x_y_vx_vy_phi(2);
    vx = x_y_vx_vy_phi(3);
    vy = x_y_vx_vy_phi(4);
    phi = x_y_vx_vy_phi(5);

end