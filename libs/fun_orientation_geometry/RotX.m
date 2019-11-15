function R = RotX(angle)
%ROTX computes the rotation matrix about world-frame X axis.
%
%   ROTX computes the elementary rotation matrix R of
%   magnitude ANGLE about the fixed X axis of the world frame.
%
%   ANGLE [rad] is the magnitude of rotation.
%
%   R is the rotation matrix.


R = eye(3);
R(2,2) = cos(angle);
R(3,3) = cos(angle);
R(2,3) = -sin(angle);
R(3,2) = sin(angle);

end