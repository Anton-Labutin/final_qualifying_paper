function [fig] = get_figure()
    
    field_param = get_field_param();
    field_height = field_param.height;
    field_width = field_param.width;

    fig = figure('Units', 'normalized', ...
                 'OuterPosition', [0 0 1 1], ...
                 'Position', [0 0 1 1]);

    ax = axes();
    ax.XLim = [0, field_width];
    ax.YLim = [0, field_height];
    ax.DataAspectRatio = [1, 1, 1];
    ax.XTick = [];
    ax.YTick = linspace(0, field_height, 21);

    fig.CurrentAxes = ax;

end