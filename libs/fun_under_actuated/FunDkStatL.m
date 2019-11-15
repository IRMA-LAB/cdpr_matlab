function [vector,matrix] = FunDkStatL(r,cdpr_p,complete_length,cable_tensions,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);

[cdpr_v,constraint_l] = CalcKinZeroOrdConstr(variables(1:3),variables(4:end),complete_length,cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_v,cdpr_p);
constraint_static = -cdpr_v.geometric_jacobian'*cable_tensions+cdpr_v.ext_load;

static_jacobian = CalcJacobianStatic(cdpr_v);
l_jacobian = CalcJacobianL(cdpr_v);
vector = [constraint_static;constraint_l];
matrix = [static_jacobian;l_jacobian];

r.SetFrame(cdpr_v,cdpr_p);

end