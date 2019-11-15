function R = RotTiltTorsion(v)
%ROTTILTTORSION computes the rotation matrix R of a sequence of two successive rotations about body fixed axes.
%
%   The two successive rotations about a and z axes of the moving frame of
%   magnitude V(2) and V(3), are respectively known as "Tilt" and "Torsion".
%   The axis a lies on the Oxy plane and its angular distances from y and x
%   axes are respectively equal to V(1) and V(1)+pi/2. The angle V(1) is
%   known as "azimuth".
%   The transformation can be thought of as a sequence of three
%   successive rotations, performed in the following order:
%   1) Rotation of magnitude V(1) about z axis of the moving frame.
%   2) Rotation of magnitude V(2) about y axis of the moving frame.
%   3) Rotation of magnitude V(3)-V(1) about z axis of the moving frame.
%
%   V is a vector (size[3,1], [rad]) containing the magnitude of the three
%   successive rotations.
%
%   R is the overall rotation matrix.


R = RotZYZ([v(1);v(2);v(3)-v(1)]);

end