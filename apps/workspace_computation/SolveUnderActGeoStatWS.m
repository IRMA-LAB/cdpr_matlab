function [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose,varargin)

if (~isempty(varargin))
    rec = varargin{1};
end

or = fsolve(@(v) FunGsNoCheck(cdpr_p,pose(1:3),v),...
    pose(4:end),ut.fsolve_options_grad);

pose = [pose(1:3);or];
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

if(isempty(cdpr_v.tension_vector(cdpr_v.tension_vector>tau_lim(2)))...
        && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<tau_lim(1))))
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
        (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateAnaliticJacobians...
        (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian);
    [~,K_matrix_an] = CalcStiffnessMatUnder(cdpr_v);
    [~,p] = chol(K_matrix_an);
    if (p==0) % stable equilibrium
        cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
        [~,M_matrix_an] = CalcMassMatUnder(cdpr_v);
        [eigenvectors_an,eigenvalues_mat_an] = eig(K_matrix_an,M_matrix_an);    
        out.counter = out.counter+1;
        out.pose(:,out.counter) = pose;
        out.position(:,out.counter) = out.pose(1:3,out.counter);
        out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
        out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
        out.nat_freq(:,out.counter) = sqrt(diag(eigenvalues_mat_an))./(2.*pi);
        out.normal_modes(:,:,out.counter) = eigenvectors_an;
        out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
    end
end

end