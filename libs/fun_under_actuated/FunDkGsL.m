function [vector,matrix] = FunDkGsL(r,cdpr_p,complete_length,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);

[cdpr_v,constraint_l] = CalcKinZeroOrdConstr(variables(1:3),variables(4:end),complete_length,cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_v.geometric_jacobian);
[cdpr_v,constraint_gs] = UnderactuatedStaticConstraint(cdpr_v);

gs_jacobian = CalcJacobianGs(cdpr_v);
l_jacobian = CalcJacobianL(cdpr_v);
vector = [constraint_gs;constraint_l];
matrix = [gs_jacobian;l_jacobian];

r.SetFrame(cdpr_v,cdpr_p);

end