function angle_d = CalcTangentAngleD(vers_n,vel_OA_glob,pos_BA_glob)
%CALCTANGENTANGLED computes the 1th order derivative of the supplementary of the wrapped angle.
%
%   VERS_N is a vector(size[3,1]), containing the components, projected on 
%   the global frame, of the versor equidirectional and equiverse to the 
%   position vector (B-C).
%   VEL_OA_GLOB is a vector (size[3,1], [m/s]), containing the components,
%   projected on the global frame, of the  1th order time derivative
%   of the position vector (A-O).
%   POS_BA_GLOB is a vector (size[3,1], [m]), containing the components of
%   the position vector (A-B), projected on the global frame.
%
%   ANGLE_D is the 1th order derivative of the supplementary of the wrapped
%   angle [rad/s].

angle_d = dot(vers_n,vel_OA_glob)/norm(pos_BA_glob);

end