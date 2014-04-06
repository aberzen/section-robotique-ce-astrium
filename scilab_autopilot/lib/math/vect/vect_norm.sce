// Norm2 of a vector
// 
// Computed as the square root of the dot product.
//
// INTPUT
// - v: a vector
//
// OUTPUT
// - res: the norm2 of the vector
//
// USAGE
// res = vect_norm(v);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [res] = vect_norm(v)
    res = sqrt(vect_dotProd(v,v));
endfunction
