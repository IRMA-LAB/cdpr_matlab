function cable_v = CalcCableVectors(sw_r,vers_k,cable_v)
%CALCCABLEVECTORS computes vectors linked to cable and swivel pulley geometry. 
%
%   CALCCABLEVECTORS computes vectors linked to geometric variables of
%   the cable and the swivel pulley storing them in a proper structure.
%   
%   SW_R is the pulley radius [m].
%   VERS_K is a vector (size[3,1]), containing the components of the unit
%   vector along the swivel axis z, projected on the global frame.
%   CABLE_V is a structure containing the time dependent variables of the 
%   cable and its swivel pulley.
%
%   TAN_ANG is supplementary to the wrapped angle of the cable inside 
%   the pulley groove.
%   VERS_N is a vector(size[3,1]), containing the components, projected on 
%   the global frame, of the versor equidirectional and equiverse to the 
%   position vector (B-C).
%   VERS_RHO is a vector(size[3,1]), containing the components, projected 
%   on the global frame, of the versor equidirectional and equiverse to 
%   the position vector (A-B).
%   POS_BA_GLOB is a vector(size[3,1],[m]), containing the components of
%   the position vectors (A-B), projected on the global frame.

cos_p = cos(cable_v.tan_ang);
sin_p = sin(cable_v.tan_ang);
cable_v.vers_n = cable_v.vers_u*cos_p + vers_k*sin_p;
cable_v.vers_t = cable_v.vers_u*sin_p - vers_k*cos_p;
cable_v.pos_BA_glob = cable_v.pos_DA_glob - sw_r*(cable_v.vers_u+cable_v.vers_n);

end