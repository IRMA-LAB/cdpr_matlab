function R = RotZ(angle)
%ROTZ computes the rotation matrix about world-frame Z axis.
%
%   ROTZ computes the elementary rotation matrix R of
%   magnitude ANGLE about the fixed Z axis of the world frame.
%
%   ANGLE [rad] is the magnitude of rotation.
%
%   R is the rotation matrix.

R = eye(3);
R(1,1) = cos(angle);
R(2,2) = cos(angle);
R(1,2) = -sin(angle);
R(2,1) = sin(angle);

end