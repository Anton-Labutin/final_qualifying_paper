function print_acceptable_maneuvers(car_1_v_grid, distance_grid, results)

    disp(' ');
    disp('Acceptable maneuvers:');
    disp(' ');
    disp('Parameters:');
    disp(' ');
    disp('car_1_v_grid:');
    disp(car_1_v_grid');
    disp(' ');
    disp('distance_grid:');
    disp(distance_grid');
    
    disp(' ');
    
    collision_str = 'COLLISION';
    success_str = 'SUCCESS';
    maneuver_needed_str = 'NEEDED';
    maneuver_not_needed_str = 'NOT NEEDED';
    
    car_1_v_grid_len = numel(car_1_v_grid);
    distance_grid_len = numel(distance_grid);
    
    for i = 1 : car_1_v_grid_len

        for j = 1 : distance_grid_len
            braking_acceptable = collision_str;
            speedup_acceptable = collision_sttr;
            change_acceptable = collision_str;
            if results{i, j}{1, 1}.ok
                braking_acceptable = success_str;
            end
            if results{i, j}{1, 2}.ok
                speedup_acceptable = success_str;
            end
            if results{i, j}{1, 3}.ok
                change_acceptable = success_str;
                curvature_range = results{i, j}{1, 3}.curvature_range;
                min_curvature = curvature_range(1);
                max_curvature = curvature_range(2);
            end
            
            braking_needed = maneuver_not_needed_str;
            speedup_needed = maneuver_not_needed_str;
            change_needed = maneuver_not_needed_str;
            if results{i, j}{1, 1}.needed 
                braking_needed = maneuver_needed_str;
            end
            if results{i, j}{1, 2}.needed
                speedup_needed = maneuver_needed_str;
            end
            if results{i, j}{1, 3}.needed
                change_needed = maneuver_needed_str;
            end
            
            disp(['car_1_v = ', num2str(car_1_v_grid(i)), ' м/с, distance = ', num2str(distance_grid(j)), ' м']);
            disp(' ');
            disp(['    braking: ', braking_needed, ', ', braking_acceptable]);
            disp(['    speedup: ', speedup_needed, ', ', speedup_acceptable]);
            
            disp(['    change : ', change_needed, ', ', change_acceptable]);
            if strcmp(change_acceptable, success_str)
                disp(['             curvature_range = [', num2str(min_curvature), ', ', num2str(max_curvature), ']']);
            end
            
            disp(' ');
            
        end
        
    end
    
end