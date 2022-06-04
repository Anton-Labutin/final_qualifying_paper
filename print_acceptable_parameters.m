function print_acceptable_parameters(car_1_braking_acceptable_v_dist, ...
                                     car_1_speedup_acceptable_v_dist, ...
                                     car_1_change_acceptable_v_dist_min_max_curv ...
                                     )

    disp(' ');
    disp('Acceptable parameters: braking');
    disp('car_1_v (м/с) | distance (м)');
    disp(car_1_braking_acceptable_v_dist);
    disp(' ');
    disp('Acceptable parameters: speedup');
    disp('car_1_v (м/с) | distance (м)');
    disp(car_1_speedup_acceptable_v_dist);
    disp(' ');
    disp('Acceptable parameters: change');
    disp('car_1_v (м/с) | distance (м) | car_1_min_curvature (1/м) | car_1_max_curvature (1/м)');
    disp(car_1_change_acceptable_v_dist_min_max_curv);
    disp(' ');

end