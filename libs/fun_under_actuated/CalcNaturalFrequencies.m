function [vector,matrix] = CalcNaturalFrequencies(cdpr_p,pose)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateAnaliticJacobians(cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

K_matrix = CalcJacobianGs(cdpr_v,cdpr_p)*cdpr_v.underactuated_platformobj.analitic_orthogonal;
M_matrix = cdpr_v.underactuated_platformobj.analitic_orthogonal'*cdpr_v.platform.mass_matrix_global_ss*cdpr_v.underactuated_platformobj.analitic_orthogonal;
for i=1:

matrix(:,cdpr_p.underactuated_platform.actuated_mask) = [];

r.SetFrame(cdpr_v,cdpr_p);



end