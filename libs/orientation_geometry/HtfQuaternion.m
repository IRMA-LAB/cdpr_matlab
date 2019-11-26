function mat = HtfQuaternion(q)
%HTFQUATERNION relates the 1th order derivatives of quaternion to angular velocity.
%   HTFQUATERNION computes the matrix that transforms the 1th order
%   time derivatives of quaternion components into the vector angular
%   velocity, projected in the fixed frame.
%
% Q is the quaternion.
%
% MAT (size[3,4]) is the transformation matrix.


mat(1,1) = -q(2);
mat(1,2) = q(1);
mat(1,3) = -q(4);
mat(1,4) = q(3);

mat(2,1) = -q(3);
mat(2,2) = q(4);
mat(2,3) = q(1);
mat(2,4) = -q(2);

mat(3,1) = -q(4);
mat(3,2) = -q(3);
mat(3,3) = q(2);
mat(3,4) = q(1);

mat=2*mat;

end