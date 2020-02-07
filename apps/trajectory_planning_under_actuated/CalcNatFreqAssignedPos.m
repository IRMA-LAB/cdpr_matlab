function f = CalcNatFreqAssignedPos(cdpr_p,cdpr_v,ut,pose)

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
cdpr_v.underactuated_platform.unactuated = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,v),...
    cdpr_v.underactuated_platform.unactuated,ut.fsolve_options_grad);

pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
K_matrix = CalcStiffnessMatUnder(cdpr_v);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
M_matrix = CalcMassMatUnder(cdpr_v);
[eigenvectors,eigenvalues_mat] = eig(K_matrix,M_matrix);
f = sqrt(diag(eigenvalues_mat))./(2.*pi);

end