function mat = HtfZYZ(angle)
%HTFZYZ relates the 1th order Euler angles derivatives to angular velocity.
%   HTFZYZ computes the matrix that transforms Euler angles 1th order
%   time derivatives into the components of the vector angular velocity,
%   projected  in the fixed frame.
%
% ANGLE is the vector (size[3,1], [rad]) containing the Euler angles.
%
% MAT (size[3,3]) is the transformation matrix.

c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = zeros(3);
mat(1,2) = -s1;
mat(1,3) = c1*s2;
mat(2,2) = c1;
mat(2,3) = s1*s2;
mat(3,1) = 1;
mat(3,3) = c2;

end