// Conjugate of a quaternion
//
// Return a quaternion with the opposite vectorial part.
//
// INTPUT
// - q: the intput quaternion
//
// OUTPUT
// - qRes: the conjugate
//
// USAGE
// qRes = quat_conj(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qRes] = quat_conj(q)
    qRes=[quat_getScal(q) ; vect_scalProd(-1,quat_getVect(q))];
endfunction
