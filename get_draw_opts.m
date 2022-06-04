function [draw_opts] = get_draw_opts()

    draw_opts = struct();

    draw_opts.draw_motion_needed = true;
    draw_opts.draw_motion_accident_needed = true;
    draw_opts.draw_motion_ok_needed = true;
    draw_opts.motion_video_needed = true;

end