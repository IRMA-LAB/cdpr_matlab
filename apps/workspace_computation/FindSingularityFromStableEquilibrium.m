function [c,ceq] = FindSingularityFromStableEquilibrium(cdpr_p,cdpr_v,ut,pose,s)

pose(4:6) = s;

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);

[cdpr_v,vector] = UnderactuatedStaticConstraintNoCheck(cdpr_v);

pose_n = NormalizeOrientation(cdpr_p,pose);

K_matrix = CalcStiffnessMatUnder(cdpr_v);
%K_tot = CalcStiffnessMatTotalUnder(cdpr_v);
%cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
%M_matrix = CalcMassMatUnder(cdpr_v);
%M_tot = CalcMassMatTotalUnder(cdpr_v);
%[eigenvectors,eigenvalues_mat] = eig(K_tot,M_tot);
d = eig(K_matrix);
%d_tot = eig(K_tot);
c = -d;
%c = [c;-cdpr_v.tension_vector]
d_abs = abs(d);
ceq = min(d_abs);
ceq = [ceq;vector]

end