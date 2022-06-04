function print_results(time_grid, car_1_motion_param, car_2_motion_param)

    arguments
        time_grid (1, :)
        car_1_motion_param (5, :) {mustBeEqualSize(time_grid, car_1_motion_param)}
        car_2_motion_param (5, :) {mustBeEqualSize(time_grid, car_2_motion_param)}
    end
    
    disp('Временная сетка: ');
    disp(time_grid);

    disp('Параметры движения 1го автомобиля: ');
    disp(car_1_motion_param);

    disp('Параметры движения 2го автомобиля: ');
    disp(car_2_motion_param);
    
end


function mustBeEqualSize(time_grid, car_motion_param_grid)

    if size(time_grid, 1) == 1
        if size(car_motion_param_grid, 1) == 5
            if size(time_grid, 2) ~= size(car_motion, 2)
                error('time_grid & car_motion_param_grid have incompatible size');
            end
        else
            if size(time_grid, 2) ~= size(car_motion, 1)
                error('time_grid & car_motion_param_grid have incompatible size');
            end
        end
    else
        if size(car_motion_param_grid, 1) == 5
            if size(time_grid, 1) ~= size(car_motion, 2)
                error('time_grid & car_motion_param_grid have incompatible size');
            end
        else
            if size(time_grid, 1) ~= size(car_motion, 1)
                error('time_grid & car_motion_param_grid have incompatible size');
            end
        end
    end

end