function [stab_result] = AssertStabilityUnderActuated(cdpr_p,pose)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateAnaliticJacobians(cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

K_matrix = CalcStiffnessMat(cdpr_v,cdpr_p);
[~,p] = chol(K_matrix);

if (p>0)
    stab_result = 0;
else
    stab_result = 1;
end