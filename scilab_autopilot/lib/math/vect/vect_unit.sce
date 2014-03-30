// Unit vector
// 
// Normalize a vector by dividing by its norm2 value.
//
// INTPUT
// - v: a vector
//
// OUTPUT
// - v_unit: the unit vector
//
// USAGE
// [v_unit] = vect_unit(v);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [v_unit] = vect_unit(v)
    v_unit = vect_scalProd(1 ./ vect_norm(v), v);
endfunction
