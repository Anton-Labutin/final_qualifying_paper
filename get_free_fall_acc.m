function [g] = get_free_fall_acc()

    g = 9.81; % ускорение свободного падения (м/с)
    assert(g >= 9 && g <= 10, 'g < 9 or g > 10');
    
end