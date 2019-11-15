function cable_v = UpdatePosA(pos_A_loc,pos_D_glob,position,rot_mat,cable_v)
%UPDATEPOSA updates each vector depending on the position of point A. 
%
%   UPDATEPOSA updates each position vectors depending on the position of 
%   the distal anchor point A and projects the components on the global
%   frame.
%   
%   POS_A_LOC is a vector(size[3,1],[m]), containing the components of the
%   position vector (A-P),projected on the local frame.
%   POS_D_GLOB is a vector(size[3,1],[m]), containing the components of the
%   position vector (D-O), projected on the global frame.
%   POSITION is a vector(size[3,1],[m]), containing the components of the
%   position vector (P-O),projected on the global frame.
%   ROT_MAT is the rotation matrix (size[3,3]).
%   CABLE_V is a structure containing time dependent cable variables. 



cable_v.pos_PA_glob = rot_mat*pos_A_loc;
cable_v.pos_OA_glob = position + cable_v.pos_PA_glob;
cable_v.pos_DA_glob = cable_v.pos_OA_glob - pos_D_glob;

end