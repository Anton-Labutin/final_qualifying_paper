function [ax] = get_axes(figure, field)
    
    field_height = field.height;
    field_width = field.width;

    ax = axes(figure);
    ax.XLim = [0, field_width];
    ax.YLim = [0, field_height];
    ax.DataAspectRatio = [1, 1, 1];
    ax.XTick = [];
    ax.YTick = linspace(0, field_height, 21);

end