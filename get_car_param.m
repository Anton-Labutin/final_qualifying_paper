% В качестве автомобиля возьмём Tesla Model 3, комплектация Performance

function [car] = get_car_param()

    car = struct();

    car.length = 4.694;    % длина (м)
    car.width = 1.849;     % ширина(м)
    car.mass = 1847;       % масса (кг)
    car.time_reaction = 0.5;  % время реации автомобиля (сек)
    car.max_speedup_acc = 5.556; % максимальное ускорение при разгоне (м/с^2)
    car.safe_dist_ahead = 1;  % безопасное расстояние до заднего бампера машины спереди (м)
    car.safe_dist_behind = car.length; % безопасное расстояние до переднего бампера машины сзади (м)
    car.safe_side_dist = 0.2; % безопасное расстояние сбоку до машин (м)
    
    assert_car_param(car);
    
end


function assert_car_param(car)

    arguments
        car (8, :) struct
    end

    length = car.length;
    width = car.width;
    mass = car.mass;
    time_reaction = car.time_reaction;
    max_speedup_acc = car.max_speedup_acc;
    safe_dist_ahead = car.safe_dist_ahead;
    safe_dist_behind = car.safe_dist_behind;
    safe_side_dist = car.safe_side_dist;
    
    assert(length > 0, 'car.length <= 0');
    assert(width > 0, 'car.width <= 0');
    assert(mass > 0, 'car.mass <= 0');
    assert(time_reaction > 0, 'car.time_reaction <= 0');
    assert(max_speedup_acc > 0, 'car.max_speedup_acc <= 0');
    assert(safe_dist_ahead > 0, 'car.safe_dist_ahead <= 0');
    assert(safe_dist_behind > 0, 'car.safe_dist_behind <= 0');
    assert(safe_side_dist > 0, 'car.safe_side_dist <= 0');

end