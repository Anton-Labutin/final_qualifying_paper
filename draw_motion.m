function draw_motion(figure, ...
                     param, ...
                     time_grid, ...
                     car_1_x_y_vx_vy_phi_grid, ...
                     car_2_x_y_vx_vy_phi_grid, ...
                     road_accident_happened, ...
                     video_needed)
                 
    if video_needed
        video_writer = get_video_writer();
        open(video_writer);
    end
    
    time_grid_len = numel(time_grid);
    
    axes = figure.CurrentAxes;
    axes_x_lim = axes.XLim;
    axes_y_lim = axes.YLim;
    axes_data_aspect_ratio = axes.DataAspectRatio;
    axes_x_tick = axes.XTick;
    axes_y_tick = axes.YTick;
    
    car_1_v_start = param.car_1_v_start;
    start_distance = param.start_distance;
    
    axes_title = ['Modelling: velocity = ', ...
                  num2str(car_1_v_start), ...
                  ' м/с, distance = ', ...
                  num2str(start_distance), ...
                  ' м'];
    
    if isfield(param, 'car_1_curvature')
        axes_title = [ axes_title, ', curvature = ', ...
                       num2str(param.car_1_curvature), ...
                       ' 1/м' ...
        ];
    end

    axes_title_font_size = 22;
    
    figure.CurrentAxes.Title.String = axes_title;
    figure.CurrentAxes.Title.FontSize = axes_title_font_size;
    
    road_param = get_road_param();
    
    lane_cnt = road_param.lane_cnt;
    lane_width = road_param.lane_width;
    road_left_side_x = road_param.left_side_x;
    
    for i = 1 : time_grid_len
        
        car_1_x = car_1_x_y_vx_vy_phi_grid(i, 1); 
        car_1_y = car_1_x_y_vx_vy_phi_grid(i, 2);
        car_1_phi = car_1_x_y_vx_vy_phi_grid(i, 5);
        
        car_1_frame = get_car_frame(car_1_x, car_1_y, car_1_phi);
        
        car_2_x = car_2_x_y_vx_vy_phi_grid(i, 1); 
        car_2_y = car_2_x_y_vx_vy_phi_grid(i, 2);
        car_2_phi = car_2_x_y_vx_vy_phi_grid(i, 5);
        
        car_2_frame = get_car_frame(car_2_x, car_2_y, car_2_phi);
    
        car_1_frame = [car_1_frame(1, :), car_1_frame(1, 1); ...
                       car_1_frame(2, :), car_1_frame(2, 1)];
    
        car_2_frame = [car_2_frame(1, :), car_2_frame(1, 1); ...
                       car_2_frame(2, :), car_2_frame(2, 1)];
    
        if i < time_grid_len
            color = 'b';
        else
            if road_accident_happened
                color = 'r';
            end
        end
    
        plot(axes, ...
            car_1_frame(1, :), car_1_frame(2, :), ...
            car_1_frame(1, 1), car_1_frame(2, 1), '.', ...
            car_1_frame(1, 2), car_1_frame(2, 2), '.', ...
            car_1_frame(1, 3), car_1_frame(2, 3), '.', ...
            car_1_frame(1, 4), car_1_frame(2, 4), '.', ...
            car_2_frame(1, :), car_2_frame(2, :), ...
            car_2_frame(1, 1), car_2_frame(2, 1), ...
            car_2_frame(1, 2), car_2_frame(2, 2), '.', ...
            car_2_frame(1, 3), car_2_frame(2, 3), '.', ...
            car_2_frame(1, 4), car_2_frame(2, 4), '.', ...
            'Color', color, 'LineWidth', 1.5 ...
        ); 
        
        car_1_vx = car_1_x_y_vx_vy_phi_grid(i, 3);
        car_1_vy = car_1_x_y_vx_vy_phi_grid(i, 4);
        car_1_v = norm([car_1_vx, car_1_vy]);
        
        car_2_vx = car_2_x_y_vx_vy_phi_grid(i, 3);
        car_2_vy = car_2_x_y_vx_vy_phi_grid(i, 4);
        car_2_v = norm([car_2_vx, car_2_vy]);
        
        str = { ...
            ['t = ', num2str(time_grid(i)), ' сек'], ...
            ['v_1 = ', num2str(car_1_v), ' м/с'], ...
            ['v_2 = ', num2str(car_2_v), ' м/с'] ...
        };
        
        text(axes, 0.65 * axes_x_lim(2), 0.85 * axes_y_lim(2), str, 'FontSize', axes_title_font_size);
    
        for j = 1 : (lane_cnt + 1)
            xline(axes, road_left_side_x + (j - 1) * lane_width);
        end
        
        axes.XLim = axes_x_lim;
        axes.YLim = axes_y_lim;
        axes.DataAspectRatio = axes_data_aspect_ratio;
        axes.XTick = axes_x_tick;
        axes.YTick = axes_y_tick;
        axes.Title.String = axes_title;
        axes.Title.FontSize = axes_title_font_size;
    
        if video_needed
            writeVideo(video_writer, getframe(figure));
        end
        
        if i < time_grid_len
            pause(2 * (time_grid(i + 1) - time_grid(i)));
        end
    end

    if video_needed
        for i = 1 : 5
            writeVideo(video_writer, getframe(figure));
        end
           
        close(video_writer);
    end
    
    pause(1);

end