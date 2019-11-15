function mat = HtfTaytBryan(angle)
%HTFTAYTBRYAN relates the 1th order derivatives of rotation angles to angular velocity.
%   HTFTAYTBRYAN computes the matrix that transforms the 1th order
%   time derivatives of rotation angles into the components of the
%   vector angular velocity, projected in the fixed frame. The angles
%   of rotation are chosen according to Tayt Bryan parameterization.
%
% ANGLE is the vector (size[3,1], [rad]) containing the angles of rotation.
%
% MAT (size[3,3]) is the transformation matrix.


c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = eye(3);
mat(1,3) = s2;
mat(2,2) = c1;
mat(2,3) = -s1*c2;
mat(3,2) = s1;
mat(3,3) = c1*c2;

end