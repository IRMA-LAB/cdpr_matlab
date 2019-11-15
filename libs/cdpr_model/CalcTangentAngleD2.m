function angle_d_2 = CalcTangentAngleD2(vers_n,vers_n_deriv,vel_OA_glob,acc_OA_glob,pos_BA_glob,vel_BA_glob,angle_d)
%CALCTANGENTANGLED2 computes the 2nd order derivative of the supplementary of the wrapped angle.
%
%   VERS_N is a vector(size[3,1]), containing the components, projected on 
%   the global frame, of the versor equidirectional and equiverse to the 
%   position vector (B-C).
%   VERS_N_DERIV is a vector(size[3,1]), containing the components, projected on 
%   the global frame, of the derivative of the versor equidirectional  
%   and equiverse to the position vector (B-C).
%   VEL_OA_GLOB is a vector (size[3,1], [m/s]), containing the components,
%   projected on the global frame, of the  1th order time derivative
%   of the position vector (A-O).
%   ACC_OA_GLOB is a vector (size[3,1], [m/s]), containing the components,
%   projected on the global frame, of the  2nd order time derivative
%   of the position vector (A-O).
%   POS_BA_GLOB is a vector (size[3,1], [m]), containing the components of
%   the position vector (A-B), projected on the global frame.
%   VEL_BA_GLOB is a vector (size[3,1], [m/s]), containing the components,
%   projected on the global frame, of the  1st order time derivative
%   of the position vector (A-B).
%   ANGLE_D is the 1th order derivative of the supplementary of the wrapped
%   angle [rad/s].
%
%   ANGLE_D_2 is the 2nd order derivative of the supplementary of the wrapped
%   angle [rad/s].

angle_d_2 = (dot(vers_n,acc_OA_glob)+dot(vers_n_deriv,vel_OA_glob)-norm(vel_BA_glob).*angle_d)/norm(pos_BA_glob);

end