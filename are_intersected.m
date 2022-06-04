function res = are_intersected(frame_1, frame_2)

    eps = 0.1;

    poly_1 = polyshape(frame_1(1, :), frame_1(2, :));
    poly_2 = polyshape(frame_2(1, :), frame_2(2, :));
    
    poly_1 = polybuffer(poly_1, eps);
    poly_2 = polybuffer(poly_2, eps);
    
    poly_intersect = intersect(poly_1, poly_2);
    
    res = (size(poly_intersect.Vertices, 1) > 0);

end