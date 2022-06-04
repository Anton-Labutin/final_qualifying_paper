function [event_names] = get_event_names()

    event_names = struct();
    
    event_names.time_limit_opts = 'time_limit_opts';
    event_names.intersection_opts = 'intersection_opts';
    event_names.motion_opts = 'motion_opts';
    event_names.collision_speedup_opts = 'collision_speedup_opts';

end