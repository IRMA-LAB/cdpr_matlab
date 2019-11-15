function R = RotY(angle)
%ROTY computes the rotation matrix about world-frame Y axis.
%
%   ROTY computes the elementary rotation matrix R of
%   magnitude ANGLE about the fixed Y axis of the world frame.
%
%   ANGLE [rad] is the magnitude of rotation.
%
%   R is the rotation matrix.

R = eye(3);
R(1,1) = cos(angle);
R(3,3) = cos(angle);
R(1,3) = sin(angle);
R(3,1) = -sin(angle);

end