function cable_v = CalcCableVectorsD(cable_p,cable_v)
%CALCCABLEVECTORSD computes the 1th order derivative of vectors linked to cable and swivel pulley geometry.
%
%   CALCCABLEVECTORSD computes the 1th order time derivative of vectors 
%   linked to geometric variables of the cable and the swivel pulley and
%   stores them in a proper structure.
%
%   CABLE_P is a structure containing static parameters of the cable and
%   the swivel pulley.
%   CABLE_V is a structure containing time dependent variables of the 
%   cable and the swivel pulley.
%
%   VERS_N_DERIV is a vector (size[3,1]), containing the components, 
%   projected on the global frame, of the 1th order time derivative of 
%   the versor equidirectional and equiverse to the position vector (B-C).
%   VEL_BA_GLOB is a vector (size[3,1]), containing the components, 
%   projected on the global frame, of the 1th order time derivative of 
%   the position vector (A-B).
%   VERS_RHO_DER is a vector (size[3,1]), containing the components of the
%   versor equidirectional and equiverse to the vector VEL_BA_GLOB.
 



cable_v.vers_n_deriv = cable_v.vers_w.*cos(cable_v.tan_ang).*cable_v.swivel_ang_vel -...
  cable_v.vers_t.*cable_v.tan_ang_vel;
cable_v.vers_t_deriv = cable_v.vers_w.*sin(cable_v.tan_ang).*cable_v.swivel_ang_vel +...
  cable_v.vers_n.*cable_v.tan_ang_vel;
cable_v.vel_BA_glob = cable_v.vel_OA_glob-cable_p.swivel_pulley_r.*...
  (cable_v.vers_w.*(1+cos(cable_v.tan_ang)).*cable_v.swivel_ang_vel - ...
  cable_v.vers_t.*cable_v.tan_ang_vel);

end