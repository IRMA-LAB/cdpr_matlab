function [vector,matrix] = FunGs(cdpr_p,act_vars,un_act_vars,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,act_vars,un_act_vars);
pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform =...
  cdpr_v.underactuated_platform.UpdateStatics(cdpr_p.underactuated_platform,...
  cdpr_v.analitic_jacobian,cdpr_v.D_mat,cdpr_v.platform.ext_load);
[cdpr_v.tension_vector,vector] = cdpr_v.underactuated_platform.CalcStaticTension(cdpr_p.underactuated_platform);

matrix = CalcJacobianGs(cdpr_v);
matrix = matrix*cdpr_p.underactuated_platform.permutation_matrix';
matrix(:,1:cdpr_p.n_cables) = [];

if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end

end