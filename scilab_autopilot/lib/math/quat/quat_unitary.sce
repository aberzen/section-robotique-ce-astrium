// Normalize the quaternion
//
// Ensure that a quaternion norm is one.
// 
// INTPUT
// - q: input quaternion
//
// OUTPUT
// - qRes: normalized quaternion (norm 1)
//
// USAGE
// qRes = quat_unitary(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qRes] = quat_unitary(q)
    n = quat_norm(q);
    qRes = [quat_getScal(q)./n ; vect_scalProd(ones(1,size(n,2))./n,quat_getVect(q))];
endfunction
