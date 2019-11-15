function [vector,matrix] = FunDkStat(r,cdpr_p,cable_tensions,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3),variables(4:end),cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_v,cdpr_p);

vector = -cdpr_v.geometric_jacobian'*cable_tensions+cdpr_v.ext_load;
matrix = CalcJacobianStatic(cdpr_v);

r.SetFrame(cdpr_v,cdpr_p);

end