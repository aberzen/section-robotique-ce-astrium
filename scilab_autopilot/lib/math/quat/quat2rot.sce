// Get rotation encoded by the quaternion
//
// INTPUT
// - q: input quaternion
//
// OUTPUT
// - ang: rotation angle (rad)
// - vectDir: unit vector around which the rotation is applied
//
// USAGE
// [ang, vectDir] = quat2rot(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [ang, vectDir] = quat2rot(q)
    ang=2*acos(quat_getScal(q));
    vectDir = vect_unitary(quat_getVect(q));
endfunction
