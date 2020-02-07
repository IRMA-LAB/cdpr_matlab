function [isposdef,pose] = SolveUnderActGeoStatNoCheckT(cdpr_p,cdpr_v,ut,pose)

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
cdpr_v.underactuated_platform.unactuated = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,v),...
    cdpr_v.underactuated_platform.unactuated,ut.fsolve_options_grad);

if norm(FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,cdpr_v.underactuated_platform.unactuated ))<0.001
    
    pose_n = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
    pose_n = NormalizeOrientation(cdpr_p,pose_n);
    
    cdpr_v = UpdateIKZeroOrd(pose_n(1:3),pose_n(4:end),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
    
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
        (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
    K_matrix = CalcStiffnessMatUnder(cdpr_v);
    d = eig(K_matrix)
    isposdef = all(d > 0);
    if (isposdef) % stable equilibrium
        pose = pose_n;
    else
        pose = pose_n;
    end
else
    isposdef = 0;
end

end