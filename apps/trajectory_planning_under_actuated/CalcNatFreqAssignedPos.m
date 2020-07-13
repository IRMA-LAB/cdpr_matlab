function f = CalcNatFreqAssignedPos(cdpr_p,cdpr_v,ut,pose)

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
[cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),fval] = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables),v),...
    cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),ut.fsolve_options_grad);
pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
if(norm(fval)<0.001 && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<0)))

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
K_matrix = CalcStiffnessMatUnder(cdpr_v);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
M_matrix = CalcMassMatUnder(cdpr_v);
[eigenvectors,eigenvalues_mat] = eig(K_matrix,M_matrix);
f = sqrt(diag(eigenvalues_mat))./(2.*pi);
end

end