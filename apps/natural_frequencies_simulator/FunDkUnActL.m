function [vector,matrix] = FunDkUnActL(cdpr_p,complete_length,un_act_vars,variables,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,variables,...
    un_act_vars);
pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

[cdpr_v,constraint_l] = CalcKinZeroOrdConstr(pose(1:3),pose(4:end),complete_length,cdpr_p,cdpr_v);

l_jacobian = CalcJacobianL(cdpr_v);
vector = constraint_l;
matrix = l_jacobian(:,1:cdpr_p.n_cables);

if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end

end