function cable_v = CalcPulleyVersors(vers_i,vers_j,cable_v)
%CALCPULLEYVERSORS computes versors connected to pulley geometry. 
%
%   The following method is based on the definition of an 
%   additional fixed right handed coordinate system, D-xyz attached to D.
%
%   VERS_I is a vector (size[3,1]), containing the components of the unit
%   vector along the x axis, projected on the global frame.
%   VERS_J is a vector (size[3,1]), containing the components of the unit
%   vector along the y axis, projected on the global frame.
%   CABLE_V is a structure containing time dependent variables of the cable
%   and the swivel pulley.
%   SWIVEL_ANG is the angle of rotation of the swivel pulley.
%
%   VERS_U is a vector (size[3,1]), containing the components, projected on 
%   the global frame, of the unit vector directed from D to C. 
%   VERS_W is a vector (size[3,1]), containing the components, projected 
%   on the global frame, of the unit vector normal to the pulley plane at 
%   the point D.

cos_s = cos(cable_v.swivel_ang);
sin_s = sin(cable_v.swivel_ang);
cable_v.vers_u = vers_i*cos_s + vers_j*sin_s;
cable_v.vers_w = -vers_i*sin_s + vers_j*cos_s;

end