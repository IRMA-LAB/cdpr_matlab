function cdpr_v = CalcExternalLoadsStateSpace(cdpr_v, cdpr_p)
%CALCEXTERNALLOADSSTATESPACE computes the external loads acting on the platform
%projected onto the state space.
%
%   CALCEXTERNALLOADS computes the components of the external loads in terms of
%   external forces and moments acting on the platform, projected onto the state space.
%
%   CDPR_P is a structure containing static parameters of the platform.
%

cdpr_v = CalcExternalLoads(cdpr_v, cdpr_p);
cdpr_v.platform.ext_load_ss(1:3, 1) = cdpr_v.platform.ext_load(1:3, 1);
cdpr_v.platform.ext_load_ss(4:6, 1) = cdpr_v.platform.H_mat' *...
  cdpr_v.platform.ext_load(4:6, 1);
