function [vector,matrix] = FunDkGsL(cdpr_p,complete_length,variables,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);

[cdpr_v,constraint_l] = CalcKinZeroOrdConstr(variables(1:3),variables(4:end),complete_length,cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateStatics(cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat,cdpr_v.platform.ext_load);
[cdpr_v.tension_vector,constraint_gs] = cdpr_v.underactuated_platform.CalcStaticTension(cdpr_p.underactuated_platform);
% cdpr_v = CalcCablesStaticTension(cdpr_v);
% [cdpr_v,constraint_gs] = UnderactuatedStaticConstraint(cdpr_v);

gs_jacobian = CalcJacobianGs(cdpr_v);
l_jacobian = CalcJacobianL(cdpr_v);
vector = [constraint_gs;constraint_l];
matrix = [gs_jacobian;l_jacobian];

if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end

end