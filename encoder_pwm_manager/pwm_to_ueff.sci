function y1 = pwm_to_ueff(u1)
    u0_pos = 0.55;
    u0_neg = 0.55;

    if u > 0 then
        if u <= u0_pos then
            y = 0;
        else
            y = (u - u0_pos) / (1 - u0_pos);
        end
    elseif u < 0 then
        if abs(u) <= u0_neg then
            y = 0;
        else
            y = (u + u0_neg) / (1 - u0_neg);
        end
    else
        y = 0;
    end
endfunction
