function [vector,matrix] = FunDkLFree(cdpr_p,complete_length,free,variables,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
pose = cdpr_p.underactuated_platform.permutation_matrix'*[variables;free];
[cdpr_v,vector] = CalcKinZeroOrdConstr(pose(1:3),pose(4:end),complete_length,cdpr_p,cdpr_v);
l_jacobian = CalcJacobianL(cdpr_v);
matrix = l_jacobian*cdpr_p.underactuated_platform.permutation_matrix';
matrix = matrix(1:cdpr_p.n_cables,1:cdpr_p.n_cables);

end
