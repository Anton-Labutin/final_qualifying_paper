function [field_param] = get_field_param()
    field_param = struct();
    
    field_param.height = 100; % длина области моделирования (м)
    field_param.width = 70; % ширина области моделирования (м)

    assert_field_param(field_param);
    
end


function assert_field_param(field)
    
    assert(field.width > 0, 'field.width <= 0');
    assert(field.height > 0, 'field.length <= 0');
    
end