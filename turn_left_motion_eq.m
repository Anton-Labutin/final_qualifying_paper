function d_x_y_vx_vy_phi_omega = turn_left_motion_eq(...
    t, x_y_vx_vy_phi_omega, R, sign_acc_rot)
    
    sign_phi = x_y_vx_vy_phi_omega(5);
    sign_omega = x_y_vx_vy_phi_omega(6);

    sign_v_linear = sign_omega * R;
    omega_squared = sign_omega^2;
    
    d_x_y_vx_vy_phi_omega = [ ...
        -sign_v_linear * sin(sign_phi);                                      % dx / dt 
        -sign_v_linear * cos(sign_phi);                                      % dy / dt
        -R * (omega_squared * cos(sign_phi) + sign_acc_rot * sin(sign_phi)); % d(v_x) / dt
        R * (omega_squared * sin(sign_phi) - sign_acc_rot * cos(sign_phi));  % d(v_y) / dt
        sign_omega;                                                          % d(phi) / dt
        sign_acc_rot ...                                                     % d(omega) / dt
    ];     

end