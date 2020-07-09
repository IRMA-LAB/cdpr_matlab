function [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose,varargin)

if (~isempty(varargin))
    rec = varargin{1};
end

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
[cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),fval] = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables),v),...
    cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),ut.fsolve_options_grad);

if norm(fval)<0.001
    
    pose_n = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
    pose_n = NormalizeOrientation(cdpr_p,pose_n);
    cdpr_v = UpdateIKZeroOrd(pose_n(1:3),pose_n(4:end),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
    if(isempty(cdpr_v.tension_vector(cdpr_v.tension_vector>tau_lim(2)))...
            && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<tau_lim(1))))
        cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
            (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
        K_matrix = CalcStiffnessMatUnder(cdpr_v);
        d = eig(K_matrix);
        isposdef = all(d) > 0;
        
        if (isposdef) % stable equilibrium
          %rec.SetFrame(cdpr_v,cdpr_p);
            cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
            M_matrix = CalcMassMatUnder(cdpr_v);
            [eigenvectors,eigenvalues_mat] = eig(K_matrix,M_matrix);
            out.counter = out.counter+1;
            out.pose(:,out.counter) = pose_n;
            out.position(:,out.counter) = out.pose(1:3,out.counter);
            out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
            out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
            out.nat_freq(:,out.counter) = sqrt(diag(eigenvalues_mat))./(2.*pi);
            out.normal_modes(:,:,out.counter) = eigenvectors;
            out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
            pose = pose_n;
        end
    end
end

end