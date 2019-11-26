function R = Quat2Rot(q)
%QUAT2ROT computes the rotation matrix R, knowing the quaternion q.
%
%
%   Q is the quaternion.
%
%   R is the resulting rotation matrix.
R = zeros(3);
R(1, 1) = q(1) * q(1) + q(2) * q(2) - q(3) * q(3) - q(4) * q(4);
R(1, 2) = 2 * (q(2) * q(3) - q(1) * q(4));
R(1, 3) = 2 * (q(2) * q(4) + q(1) * q(3));
R(2, 1) = 2 * (q(2) * q(3) + q(1) * q(4));
R(2, 2) = q(1) * q(1) - q(2) * q(2) + q(3) * q(3) - q(4) * q(4);
R(2, 3) = 2 * (q(3) * q(4) - q(1) * q(2));
R(3, 1) = 2 * (q(2) * q(4) - q(1) * q(3));
R(3, 2) = 2 * (q(3) * q(4) + q(1) * q(2));
R(3, 3) = q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4);

end