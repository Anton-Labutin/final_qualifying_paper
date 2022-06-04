function [video_writer] = get_video_writer()

    video_writer = VideoWriter('tmp.avi'); 
    video_writer.FrameRate = 10;

end