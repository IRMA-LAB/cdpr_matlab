function [vector,matrix] = FunGsNoCheck(cdpr_p,act_vars,un_act_vars,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,act_vars,un_act_vars);
pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);

[cdpr_v,vector] = UnderactuatedStaticConstraintNoCheck(cdpr_v);

matrix = CalcJacobianGs(cdpr_v,cdpr_p);
matrix(:,cdpr_p.underactuated_platform.actuated_mask) = [];

if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end



end