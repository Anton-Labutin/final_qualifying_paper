function sol = lane_change(turn_1_motion_param, ...
                           turn_2_motion_param, ...
                           opts, ...
                           change_right)
    arguments
        turn_1_motion_param (1, 7) struct {mustBeNonempty} % {x_0, y_0, v_0, phi_0, acc_rot_1, t_1, R_1}
        turn_2_motion_param (1, 3) struct {mustBeNonempty} % {t_3, R_3, acc_rot_3}
        opts (:, :) struct
        change_right (1, 1) logical {mustBeNumericOrLogical} = false
    end
    
    print_opts = get_print_opts();
    print_headers = print_opts.print_headers;
    
    if print_headers
        disp('START lane_change');
    end
      
    % поворот по дуге окружности радиуса R_1 с угловым ускорением
    % sign_acc_rot_1 из позиции (x_0, y_0) с начальной скоростью v_0 под
    % углом phi_0 к оси OY за время t_1
    
    field_names = get_field_names();
    
    if ~change_right
        % поворот налево
        sol_1 = turn_left(turn_1_motion_param, opts);
    else
        % поворот направо
        sol_1 = turn_right(turn_1_motion_param, opts);
    end
    
    t_grid_1 = (sol_1.x)';
    x_y_vx_vy_phi_grid_1 = (sol_1.y)';
    
    if (~isfield(sol_1, 'xe')) || (numel(sol_1.xe) == 0)
            
            % поворот по дуге окружности радиуса R_3 с угловым
            % ускорением sign_acc_rot_3 за время t_3
    
            t = getfield(turn_2_motion_param, field_names.t);
            % time_start = turn_2_motion_param.time_start;
            R = getfield(turn_2_motion_param, field_names.R);
            acc_rot = getfield(turn_2_motion_param, field_names.acc_rot);
            
            % v_0 = norm([x_y_vx_vy_phi_omega_grid_2(end, 3), x_y_vx_vy_phi_omega_grid_2(end, 4)]);
            v_0 = norm([x_y_vx_vy_phi_grid_1(end, 3), x_y_vx_vy_phi_grid_1(end, 4)]);
            
            
            turn_2_motion_param = struct(field_names.x, x_y_vx_vy_phi_grid_1(end, 1), ...
                                  field_names.y, x_y_vx_vy_phi_grid_1(end, 2), ...
                                  field_names.v, v_0, ...
                                  field_names.phi, x_y_vx_vy_phi_grid_1(end, 5), ...
                                  field_names.t, t, ...
                                  field_names.acc_rot, acc_rot, ...
                                  field_names.R, R);

            if ~change_right
                % поворот направо
                sol_3 = turn_right(turn_2_motion_param, opts);
            else
                % поворот налево
                sol_3 = turn_left(turn_2_motion_param, opts);
            end
            
            t_grid_3 = (sol_3.x)';
            x_y_vx_vy_phi_grid_3 = (sol_3.y)';
            
            % t_grid = [t_grid_1; t_grid_2(2 : end); t_grid_3(2 : end)];
            % x_y_vx_vy_phi_omega_grid = [x_y_vx_vy_phi_omega_grid_1; ...
            %                          x_y_vx_vy_phi_omega_grid_2(2 : end, :); ...
            %                          x_y_vx_vy_phi_omega_grid_3(2 : end, :)];
            
            t_grid = [t_grid_1; t_grid_3(2 : end)];
            x_y_vx_vy_phi_grid = [x_y_vx_vy_phi_grid_1; ...
                                        x_y_vx_vy_phi_grid_3(2 : end, :)];
            
            sol.x = (t_grid)';
            sol.y = (x_y_vx_vy_phi_grid)';
%             sol.x = [sol_1.x; ...
%                      sol_2.x(2 : end); ...
%                      sol_3.x(2 : end)];
%             sol.y = [sol_1.y; ...
%                      sol_2.y(2 : end, :); ...
%                      sol_3.y(2 : end, :)];
%            
            if isfield(sol_3, 'xe')
                sol.xe = sol_3.xe;
                sol.ye = sol_3.ye;
                sol.ie = sol_3.ie;
            end
%         else
%             t_grid = [t_grid_1; t_grid_2(2 : end)];
%             x_y_vx_vy_phi_omega_grid = [x_y_vx_vy_phi_omega_grid_1; ...
%                                         x_y_vx_vy_phi_omega_grid_2(2 : end, :)];
%                                     
%             sol.x = (t_grid)';
%             sol.y = (x_y_vx_vy_phi_omega_grid)';
%         
% %             sol.x = [sol_1.x; ...
% %                      sol_2.x(2 : end)];
% %             sol.y = [sol_1.y; ...
% %                      sol_2.y(2 : end, :)];
%             sol.xe = sol_2.xe;
%             sol.ye = sol_2.ye;
%             sol.ie = sol_2.ie;
%         end
    else
        t_grid = t_grid_1;
        x_y_vx_vy_phi_grid = x_y_vx_vy_phi_grid_1;
        
        % sol.x = sol_1.x;
        % sol.y = sol_1.y;
        sol.x = (t_grid)';
        sol.y = (x_y_vx_vy_phi_grid)';
        
        sol.xe = sol_1.xe;
        sol.ye = sol_1.ye;
        sol.ie = sol_1.ie;
    end
    
    if print_headers
        disp('FINISH lane_change');
    end
    
end


% function [t_grid, x_y_vx_vy_phi_omega_grid] = lane_change_circle_1(x_0, y_0, v_0, lane_width, change_right, rot_angle, ode_opts)
%     R = lane_width / (3 * (1 + cos(pi - rot_angle)));
%     omega_abs = v_0 / R;
%     t = rot_angle / omega_abs;
%     
%     if ~change_right
%         phi_0 = pi;
%         omega_0 = -omega_abs;
%     else
%         phi_0 = 0;
%         omega_0 = omega_abs;
%     end
%     
%     [t_grid, x_y_vx_vy_phi_omega_grid] = ...
%             ode45(@(t, xy) combined_motion_eq(t, xy, 0, 0, 0, R, 0), ...
%             [0, t], ...
%             [x_0, y_0, 0, v_0, phi_0, omega_0], ...
%             ode_opts);
%         
%     if ~change_right
%         x_y_vx_vy_phi_omega_grid(:, 5) = x_y_vx_vy_phi_omega_grid(:, 5) + pi;
%     end
% end


% function [t_grid, x_y_vx_vy_phi_omega_grid] = lane_change_circle_2(x_0, y_0, vx_0, vy_0, phi_0, lane_width, change_right, ode_opts)
%     v_0 = norm([vx_0, vy_0]);
%     
%     R = lane_width / (3 * (1 - cos(phi_0)));
%     omega_abs = v_0 / R;
%     
%     if ~change_right
%         phi_start = phi_0;
%         t = (2 * pi - phi_0) / omega_abs;
%         omega_0 = omega_abs;
%     else
%         phi_start = phi_0 + pi;
%         t = phi_0 / omega_abs;
%         omega_0 = -omega_abs;
%     end
%         
%     [t_grid, x_y_vx_vy_phi_omega_grid] = ode45(@(t, xy) combined_motion_eq(t, xy, 0, 0, 0, R, 0), ...
%         [0, t], ...
%         [x_0, y_0, vx_0, vy_0, phi_start, omega_0], ...
%         ode_opts);
%     
%     if change_right
%         x_y_vx_vy_phi_omega_grid(:, 5) =  x_y_vx_vy_phi_omega_grid(:, 5) - pi;
%     end
% end