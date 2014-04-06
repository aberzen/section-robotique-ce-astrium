// Cross product
// 
// Only meaningful on R3.
//
// INTPUT
// - v1: a R3 vector
// - v2: another R3 vector
//
// OUTPUT
// - res: the cross product
//
// USAGE
// res = vect_crossProd(v1, v2);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [res] = vect_crossProd(v1, v2)
    res = [ v1(2,:).*v2(3,:) - v1(3,:).*v2(2,:) ; v1(3,:).*v2(1,:) - v1(1,:).*v2(3,:) ; v1(1,:).*v2(2,:) - v1(2,:).*v2(1,:) ];
endfunction
