function cdpr_v = CalcDynamicLoadsStateSpace(cdpr_v,cdpr_p)
%DYN_LOAD computes the the dynamic loads acting on the platform.
%
%   DYN_LOAD computes the components of the dynamic loads in terms of
%   dynamic forces and moments acting on the platform.
%
%   PLATFORM_V is a structure containing time dependent variables of the
%   platform.
%   DYN_LOAD_SS is a vector (size[6,1]), containing the components of the
%   dynamic loads, projected on the state space.

  cdpr_v = CalcDynamicLoads(cdpr_v,cdpr_p);
  anti_com = Anti(cdpr_v.platform.pos_PG_glob);
  cdpr_v.platform.dyn_load_ss(1:3,1) = cdpr_v.platform.dyn_load(1:3,1)-...
    cdpr_p.platform.mass.*anti_com*cdpr_v.platform.H_mat_deriv*...
    cdpr_v.platform.orientation_deriv;
  cdpr_v.platform.dyn_load_ss(4:6,1) = cdpr_v.platform.H_mat'*(cdpr_v.platform.dyn_load(4:6,1)+...
    cdpr_v.platform.inertia_matrix_global*cdpr_v.platform.H_mat_deriv*...
    cdpr_v.platform.orientation_deriv);

end