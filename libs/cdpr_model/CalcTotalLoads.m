function cdpr_v = CalcTotalLoads(cdpr_v,cdpr_p)
%TOT_LOAD computes the the total loads acting on the platform.
%
%   TOT_LOAD performs the summetion of external and dynamic loads acting
%   on the platform.
%
%   CDPR_V is a structure containing the variables associated with the CDPR
%   TOT_LOAD is a vector (size[6,1]), containing the components of the
%   total loads, projected on the global frame.
  cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
  cdpr_v = CalcDynamicLoads(cdpr_v,cdpr_p);
  cdpr_v.total_load = cdpr_v.ext_load-cdpr_v.dyn_load;

end