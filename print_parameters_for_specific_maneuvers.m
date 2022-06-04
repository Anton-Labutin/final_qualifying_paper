function print_parameters_for_specific_maneuvers( ...
    car_1_v_grid, distance_grid, results, maneuvers_values ...
    )

    braking_value = maneuvers_values.braking;
    speedup_value = maneuvers_values.speedup;
    change_value = maneuvers_values.change;
    
    braking_str = 'COLLISION';
    speedup_str = 'COLLISION';
    change_str = 'COLLISION';
    
    if braking_value
        braking_str = 'SUCCESS';
    end
    if speedup_value
        speedup_str = 'SUCCESS';
    end
    if change_value
        change_str = 'SUCCESS';
    end

    disp('Acceptable parameters for specific values: ');
    disp(['    braking: ', braking_str, ', speedup: ', speedup_str, ', change: ', change_str]);
    disp(' ');
    
    car_1_v_grid_len = numel(car_1_v_grid);
    distance_grid_len = numel(distance_grid);
    
    if change_value == true
        disp('car_1_v ( м/с) | distance (м) | min_curvature (1/м) | max_curvature (1/м)');
    else
        disp('car_1_v (м/с) | distance (м)');
    end
    
    for i = 1 : car_1_v_grid_len
        for j = 1 : distance_grid_len 
            cur_cell = results{i, j};
            
            if ((cur_cell{1, 1}.ok == braking_value) && ...
                (cur_cell{1, 2}.ok == speedup_value) && ...
                (cur_cell{1, 3}.ok == change_value))
            
                if change_value == true
                    curvature_range = cur_cell{1, 3}.curvature_range;
                    min_curvature = curvature_range(1);
                    max_curvature = curvature_range(2);
                    
                    disp([...
                          num2str(car_1_v_grid(i)), '    ', ...
                          num2str(distance_grid(j)), '    ', ...
                          num2str(min_curvature), '    ', ...
                          num2str(max_curvature) ...
                        ]);
                else
                    disp([ ...
                          num2str(car_1_v_grid(i)), '    ', ...
                          num2str(distance_grid(j)) ...
                          ]);
                end 
            end
        end
    end
    
    disp(' ');

end