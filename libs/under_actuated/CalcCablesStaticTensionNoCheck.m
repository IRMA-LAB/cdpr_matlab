function cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v)
%CALCCABLESTENSION computes the cables tension under static equilibrium conditions.
%
%   ANALITIC_JACOBIAN is the analitic jacobian matrix (size[nc,6]), where
%   nc is the number of cables.
%   EXT_LOAD is a vector(size[6,1]), containing the components of the
%   external forces and moments acting on the platform.
%
%   VECTOR is a vector(size[nc,1]), containing the tension values acting
%   on each cable.
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.

cdpr_v.tension_vector = linsolve(cdpr_v.geometric_jacobian*cdpr_v.geometric_jacobian',...
    cdpr_v.geometric_jacobian*cdpr_v.platform.ext_load);


end