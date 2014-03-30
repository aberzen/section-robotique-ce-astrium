function [res] = vect_scalProd(lambda, vect)
    res = [ lambda .* vect(1,:) ; lambda .* vect(2,:) ; lambda .* vect(3,:) ];
endfunction
