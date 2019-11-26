function angle_d_2 = CalcSviwelAngleD2(vers_u,vers_w,vers_w_deriv,vel_OA_glob,acc_OA_glob,pos_DA_glob)
%CALCSWIVELANGLED2 computes the 2nd order derivative of the rotation angle of the swivel pulley.
%
%   VERS_U is a vector (size[3,1]), containing the components, projected on 
%   the global frame, of the unit vector directed from D to C.
%   VERS_W is a vector (size[3,1]), containing the components, projected on
%   the global frame, of the unit vector normal to the pulley plane at the
%   point D.
%   VERS_W_DERIV is a vector (size[3,1]), containing the components, projected on
%   the global frame, of the time derivative unit vector normal to the
%   pulley plane at the point D.
%   VEL_OA_GLOB is a vector (size[3,1], [m/s]), containing the components
%   of the 1th order time derivative of the position vector (A-O), 
%   projected on the global frame.
%   ACC_OA_GLOB is a vector (size[3,1], [m/s]), containing the components
%   of the 2nd order time derivative of the position vector (A-O), 
%   projected on the global frame.
%   POS_DA_GLOB is a vector (size[3,1], [m]), containing the components
%   of the position vector (A-D), projected on the global frame.
%
%   ANGLE_D_2 is the 2nd order time derivative of the swivel angle.   

angle_d_2 = (dot(vers_w,acc_OA_glob)-2.*dot(vers_w_deriv,vel_OA_glob))/dot(vers_u,pos_DA_glob);

end