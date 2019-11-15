function R = RotZYZ(v)
%ROTZYZ computes the rotation matrix R of a sequence of three successive rotations about the moving frame.
%
%   Rotations are performed in the following order:
%   1) Rotation of magnitude V(1) about z axis of the moving frame.
%   2) Rotation of magnitude V(2) about y axis of the moving frame.
%   3) Rotation of magnitude V(3) about z axis of the moving frame.
%
%   V is a vector (size[3,1], [rad]) containing the magnitude of the three
%   successive rotations, known as "Euler angles".
%
%   R is the overall rotation matrix.

R = RotZ(v(1))*RotY(v(2))*RotZ(v(3));

end