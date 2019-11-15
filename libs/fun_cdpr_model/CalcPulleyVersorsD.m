function cable_v = CalcPulleyVersorsD(cable_v)
%CALCPULLEYVERSORSD computes the 1th order derivative of versors connected to pulley geometry.
%
%   CABLE_V is a structure containing time dependent variables of the cable
%   and the swivel pulley.
%   SWIVEL_ANG_VEL is the 1th order time derivative [rad/s], of the angle
%   of rotation of the swivel pulley.
%
%   VERS_U_DERIV is a vector (size[3,1]), containing the components, 
%   projected on the global frame,  of the 1th order time derivative of 
%   the unit vector U.
%   VERS_W_DERIV is a vector (size[3,1]), containing the components, 
%   projected on the global frame,  of the 1th order time derivative of 
%   the unit vector W.

cable_v.vers_u_deriv = cable_v.vers_w.*cable_v.swivel_ang_vel;
cable_v.vers_w_deriv = -cable_v.vers_u.*cable_v.swivel_ang_vel;

end