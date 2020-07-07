function R = RotXYZ(v)
%ROTXYZ computes the rotation matrix R of a sequence of three successive rotations about the fixed frame.
%
%   ROTXYZ computes the rotation matrix R of a sequence of three
%   successive rotations, performed in the following order:
%   1) Rotation of magnitude v(3) about Z axis of the fixed frame
%   2) Rotation of magnitude v(2) about Y axis of the fixed frame
%   3) Rotation of magnitude v(1) about X axis of the fixed frame
%
%   V is a vector (size[3,1], [rad]) containing the magnitude of the three
%   successive rotations.
%
%   R is the overall rotation matrix.

R = RotX(v(1))*RotY(v(2))*RotZ(v(3));

end