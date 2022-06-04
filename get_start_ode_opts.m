function [ode_opts] = get_start_ode_opts()
    
    eps = get_eps();
    
    motion_time_param = get_motion_time_param();
    time_step = motion_time_param.step;
    
    ode_opts = odeset('RelTol', eps, ...
                      'AbsTol', eps, ...
                      'InitialStep', time_step, ...
                      'MaxStep', time_step);

end