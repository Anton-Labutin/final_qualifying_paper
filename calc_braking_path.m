function [braking_path] = calc_braking_path(v_start, v_finish, abs_acc)

    arguments
        v_start (1, 1) double {mustBePositive}
        v_finish (1, 1) double {mustBeNonnegative, mustBeLessThanVStart(v_start, v_finish)}
        abs_acc (1, 1) double {mustBePositive}
    end

    braking_path = 0.5 * (v_start^2 - v_finish^2) / abs_acc;

end


function mustBeLessThanVStart(v_start, v_finish)

    if v_start <= v_finish
        error('v_start <= v_finish');
    end

end