function out = CalcNatFreqPose(cdpr_p,cdpr_v,pose)

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
cdpr_v = CalcCablesStaticTension(cdpr_v);
K_matrix = CalcStiffnessMatUnder(cdpr_v);
d = eig(K_matrix);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
M_matrix = CalcMassMatUnder(cdpr_v);
[eigenvectors,eigenvalues_mat] = eig(K_matrix,M_matrix);
out = sqrt(diag(eigenvalues_mat))./(2.*pi);

end