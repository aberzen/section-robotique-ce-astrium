// Compute the norm of a quaternion
//
// INTPUT
// - q: the input quaternion
//
// OUTPUT
// - n: the norm of the input quaternion
//
// USAGE
// n = quat_norm(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [n] = quat_norm(q)
    n = sqrt(quat_getScal(quat_mult(q,quat_conj(q))));
endfunction
