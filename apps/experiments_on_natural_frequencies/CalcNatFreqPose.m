function f = CalcNatFreqPose(cdpr_p,cdpr_v,pose)

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
K_tot = CalcStiffnessMat(cdpr_v);
K_ort = CalcStiffnessMatOrt(cdpr_v.underactuated_platform.geometric_orthogonal,K_tot);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
M_matrix = CalcMassMatUnder(cdpr_v);
[~,eigenvalues_mat] = eig(K_ort,M_matrix);
f = sqrt(diag(eigenvalues_mat))./(2*pi);

end