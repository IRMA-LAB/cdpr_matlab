function cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p)
%CALCEXTERNALLOADS computes the external loads acting on the platform.
%
%   CALCEXTERNALLOADS computes the components of the external loads in terms of
%   external forces and moments acting on the platform.
%
%   PLATFORM_P is a structure containing static parameters of the platform.
%   R_MAT is the rotation matrix (size[3,3]).
%   POS_PG_GLOB is a vector(size[3,1], [m]), containing the components of
%   the position vector (G-P), projected on the global frame.
%   R is a matrix (size[6,6]), that premultiplies the equation of dynamic
%   equilibrium in order to make the mass matrix symmetric.
%     
%   EXT_LOAD is a vector([6,1]), containing the components of external
%   forces and moments, projected on the global frame.

  cdpr_v.platform.ext_load(1:3,1) = cdpr_p.platform.mass.*cdpr_p.platform.gravity_acceleration + ...
    cdpr_v.platform.rot_mat*cdpr_p.platform.ext_force_loc+cdpr_p.platform.ext_force_glob;
  cdpr_v.platform.ext_load(4:6,1) = Anti(cdpr_v.platform.pos_PG_glob)*cdpr_v.platform.ext_load(1:3,1) + ...
    cdpr_v.platform.rot_mat*cdpr_p.platform.ext_torque_loc+cdpr_p.platform.ext_torque_glob;

end