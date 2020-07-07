function mat = DHtfZYZ(angle,angle_d)
%DHTFZYZ computes the 1th order time derivative of the matrix HTFZYZ.
%
%   ANGLE is the vector (size[3,1], [rad]) containing the Euler angles.
%   ANGLE_D is the vector (size[3,1], [rad]) containing the 1th order
%   time derivatives of Euler angles.
%
%   MAT (size[3,3]) is the resulting transformation matrix.

c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = zeros(3);
mat(1,2) = -c1*angle_d(1);
mat(1,3) = -s1*s2*angle_d(1) + c1*c2*angle_d(2);
mat(2,2) = -s1*angle_d(1);
mat(2,3) = c1*s2*angle_d(1) + s1*c2*angle_d(2);
mat(3,3) = -s2*angle_d(2);

end