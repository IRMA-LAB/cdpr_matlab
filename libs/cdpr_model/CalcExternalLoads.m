function cdpr_v = CalcExternalLoads(cdpr_v, cdpr_p)
%CALCEXTERNALLOADS computes the external loads acting on the platform.
%
%   CALCEXTERNALLOADS computes the components of the external loads in terms of
%   external forces and moments acting on the platform.
%
%   CDPR_P is a structure containing static parameters of the platform.
%   TO BE COMPLETED..

cdpr_v.platform.ext_load(1:3, 1) = cdpr_p.platform.mass .* ...
  cdpr_p.platform.gravity_acceleration + cdpr_v.platform.rot_mat * ...
  cdpr_p.platform.ext_force_loc + cdpr_p.platform.ext_force_glob;

cdpr_v.platform.ext_load(4:6, 1) = Anti(cdpr_v.platform.pos_PG_glob) * ...
  cdpr_v.platform.ext_load(1:3,1) + cdpr_v.platform.rot_mat * ...
  cdpr_p.platform.ext_torque_loc + cdpr_p.platform.ext_torque_glob;
