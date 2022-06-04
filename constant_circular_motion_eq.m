function d_x_y_vx_vy_phi = constant_circular_motion_eq(t, y, omega, R, phi_0)
    d_x_y_vx_vy_phi = [ omega * R * sin(y(5)); % координата x
                 omega * R * cos(y(5)); % координата y
                 omega^2 * R * cos(y(5)); % скорость vx
                 -omega^2 * R * sin(y(5)); % скорость vy
                 omega ]; % угол от pi (по часовой стрелке > 0)
end