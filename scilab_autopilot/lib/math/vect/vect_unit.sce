function [v_unit] = vect_unit(v)
    v_unit = vect_scalProd(1 ./ vect_norm(v), v);
endfunction
