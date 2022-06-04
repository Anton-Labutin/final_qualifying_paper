function [motion_time] = get_motion_time_param()

    motion_time = struct();
    
    motion_time.start = 0;                                             % время начала моделирования (сек)
    motion_time.finish = 6;                                            % время конца моделирования (сек)
    motion_time.change = motion_time.start + ...
        (motion_time.finish - motion_time.start) / 4;                  % время начала перестроения 2го автомобиля (сек)
    motion_time.step = 0.05;
    
    assert_motion_time_param(motion_time);
    
end


function assert_motion_time_param(time)

    assert(time.start >= 0, 'motion_time.start < 0');
    assert(time.finish > time.start, ...
            'motion_time.finish <= motion_time.start');
    assert(time.change >= time.start, ...
            'motion_time.change < motion_time.start');
    assert(time.change < time.finish, ...
            'motion_time.change >= motion_time.finish');
    assert(time.step > 0, 'motion_time.step <= 0');
    assert(time.step < (time.finish - time.start), ...
            'motion_time.step >= (motion_time.finish - motion_time.start)');

end