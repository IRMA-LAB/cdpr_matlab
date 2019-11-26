function angle_d = CalcSviwelAngleD(vers_u,vers_w,vel_OA_glob,pos_DA_glob)
%CALCSWIVELANGLED computes the 1th order derivative of the rotation angle of the swivel pulley.
%
%   VERS_U is a vector (size[3,1]), containing the components, projected on 
%   the global frame, of the unit vector directed from D to C.
%   VERS_W is a vector (size[3,1]), containing the components, projected on
%   the global frame, of the unit vector normal to the pulley plane at the
%   point D.
%   VEL_OA_GLOB is a vector (size[3,1], [m/s]), containing the components
%   of the 1th order time derivative of the position vector (A-O), 
%   projected on the global frame.
%   POS_DA_GLOB is a vector (size[3,1], [m]), containing the components
%   of the position vector (A-D), projected on the global frame.
%
%   ANGLE_D is the 1th order time derivative of the swivel angle.   

angle_d = dot(vers_w,vel_OA_glob)/dot(vers_u,pos_DA_glob);

end