function MakeVideo(ss,filename,video_flag,par,var,rec,ut)

if video_flag
    n_frames = floor(ut.frame_rate*ss.t(end));
    frame_counter = 0;
    for j = 1:length(ss.t)
        if (j >= frame_counter*ut.sampling_count)
            frame_counter = frame_counter +1;
            var = UpdateIKZeroOrd(ss.platform.pose(1:3,j),ss.platform.pose(4:end,j),par,var);
            rec.SetFrame(var,par);
            frames(frame_counter) = getframe(rec.figure_handle);
            if (frame_counter == n_frames)
                break;
            end
        end
    end
    myVideo = VideoWriter(filename);
    myVideo.FrameRate = ut.frame_rate;
    open(myVideo)
    writeVideo(myVideo,frames);
    close(myVideo);
    clear frames
end

end