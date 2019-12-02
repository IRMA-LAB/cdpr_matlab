function q = Rot2Quat(R)
%ROT2QUAT computes the quaternion components, knowing the rotation matrix R.
%
%   The algorithm computes the quadratic value of each quaternion
%   components, arranged in Q_SQUARE. Quaternion theory ensures
%   Q_SQUARE maximum component is a nonzero value, thus it is
%   possibile to divide proper expressions by it in order to
%   evaluate the remaining components. Aforementioned expressions
%   involve rotation matrix elements and change according to the
%   position of the maximum value in the Q_SQUARE array.
%
%   R is the rotation matrix.
%
%   Q is the resulting quaternion.

q_square(1) = 0.25 * (1 + R(1,1) + R(2,2) + R(3,3)); %=q1^2
q_square(2) = 0.25 * (1 + R(1,1) - R(2,2) - R(3,3)); %=q2^2
q_square(3) = 0.25 * (1 - R(1,1) + R(2,2) - R(3,3)); %=q3^2
q_square(4) = 0.25 * (1 - R(1,1) - R(2,2) + R(3,3)); %=q4^2
[~,i] = max(q_square);

switch (i)
    case 1
        q(1,1) = sqrt(q_square(1));
        q(2,1) = 0.25 * (R(3,2) - R(2,3)) / q(1);
        q(3,1) = 0.25 * (R(1,3) - R(3,1)) / q(1);
        q(4,1) = 0.25 * (R(2,1) - R(1,2)) / q(1);
    case 2
        q(2,1) = sqrt(q_square(2));
        q(1,1) = 0.25 * (R(3,2) - R(2,3)) / q(2);
        q(3,1) = 0.25 * (R(1,2) + R(2,1)) / q(2);
        q(4,1) = 0.25 * (R(1,3) - R(3,1)) / q(2);
    case 3
        q(3,1) = sqrt(q_square(3));
        q(1,1) = 0.25 * (R(1,3) - R(3,1)) / q(3);
        q(2,1) = 0.25 * (R(1,2) + R(2,1)) / q(3);
        q(4,1) = 0.25 * (R(2,3) + R(3,2)) / q(3);
    case 4
        q(4,1) = sqrt(q_square(4));
        q(1,1) = 0.25 * (R(2,1) - R(1,2)) / q(4);
        q(2,1) = 0.25 * (R(1,3) + R(3,1)) / q(4);
        q(3,1) = 0.25 * (R(2,3) + R(3,2)) / q(4);
end