function [field_names] = get_field_names()
    field_names = struct();
    
    field_names.x = 'x';
    field_names.y = 'y';
    
    field_names.v = 'v';
    field_names.acc_tan = 'acc_tan';
    field_names.phi = 'phi';
    
    field_names.acc_rot = 'acc_rot';
    field_names.R = 'R';
    
    field_names.t = 't';
   
    field_names.eps = 'eps';
    
    field_names.length = 'length';
    field_names.width = 'width';
end

