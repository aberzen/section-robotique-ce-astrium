// Define positive a quaternion
//
// This function ensures that the scalar part is positive.
// This function is typically used to ensure sign of vectorial part
// when considering the product of two quaternions with a small angles
// approximation.
// 
// INTPUT
// - qIn: input quaternion
//
// OUTPUT
// - qOut: quaternion with positive scalar part
//
// USAGE
// qOut = quat_definePositive(qIn);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qOut] = quat_definePositive(qIn)
    qOut = [sign(qIn(1,:)).*qIn(1,:) ; sign(qIn(1,:)).*qIn(2,:) ; sign(qIn(1,:)).*qIn(3,:) ; sign(qIn(1,:)).*qIn(4,:)];
endfunction
