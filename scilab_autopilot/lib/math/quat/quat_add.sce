// Addition of two quaternion
//
// INTPUT
// - q1: first quaternion
// - q2: second quaternion
//
// OUTPUT
// - qRes: sum of the two quaternions
//
// USAGE
// qRes = quat_add(q1, q2);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qRes] = quat_add(q1, q2)
    qRes=q1+q2;
    [qRes] = quat_definePositive(qRes);
endfunction
