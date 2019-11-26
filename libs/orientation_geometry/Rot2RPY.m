function angle = Rot2RPY(R,aPrev)
%ROT2RPY computes the magnitude of the rotations that result the matrix R.
%
%   Moreover the three successive rotations are performed in the
%   following order:
%   1) Rotation of magnitude V(1) about z axis of the moving frame.
%   2) Rotation of magnitude V(2) about y axis of the moving frame.
%   3) Rotation of magnitude V(3) about x axis of the moving frame.
%
%   R is the rotation matrix.
%   APREV is the vector (size[3,1], [rad]) containing the magnitude of the
%   three successive rotations, referred to the previous instant time.
%
%   V is the vector (size[3,1], [rad]) containing the magnitude of the three
%   successive rotations, respectively known as "yaw","pitch","roll".

angle(2,1) = atan2(-R(3,1),(sqrt(R(3,2)^2+R(3,3)^2)+sqrt(R(1,1)^2+R(2,1)^2))/2);

if cos(angle(2,1)) >= 0.0001 || cos(angle(2,1)) <= -0.0001
    angle(1,1) = atan2(R(3,2),R(3,3));
    angle(3,1) = atan2(R(2,1),R(1,1));
else
    % Singularity of parametrization: cos(pitch)=0
    diff = atan2((-R(1,2)+R(2,3))/2,(R(1,3)+R(2,2))/2);
    angle(3,1) = aPrev(3);
    angle(1,1) = diff + angle(3);
end
end