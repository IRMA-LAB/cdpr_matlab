function val = OptimExcitFunction(cdpr_p,cdpr_v,ws_par,param,ut)

l = param(1:cdpr_p.n_cables,1);
in_cond = param(cdpr_p.n_cables+1:end,1);
pose0 = FindClosestPose(cdpr_p,cdpr_v,ws_par,l);

pose = fsolve(@(v) FunDkGsL(cdpr_p,l,v),pose0,ut.fsolve_options_grad);

T_max = 10;
act_pose_0 = cdpr_p.underactuated_platform.permutation_matrix*pose;
free_pose_0 = act_pose_0(cdpr_p.n_cables+1:end);
act_pose_0 = act_pose_0(1:cdpr_p.n_cables);
sol = ode45(@(time,free_pose) IntegrableFreeDynamics(cdpr_p,cdpr_v,...
    time,free_pose,act_pose_0,l,ut.fsolve_options_grad),[0 T_max],...
    [in_cond(1:length(in_cond)/2)+free_pose_0;in_cond(length(in_cond)/2+1:end)]);
t = linspace(0,T_max,101);
[y yp] = deval(sol,t);
sis = length(free_pose_0);
free_p = y(1:sis,:);
free_v = y(sis+1:end,:);
free_a = yp(sis+1:end,:);
%     ampl_phase = fsolve(@(v) FindAmplPhase(eigenvectors,f,v,in_cond),zeros(2*length(f),1),ut.fsolve_options_nopar);
%     A = ampl_phase(1:length(ampl_phase)/2);
%     phase = ampl_phase(length(ampl_phase)/2+1:end);
%     [dpf,dvf,daf] = ComputeExcitation(cdpr_p,A,eigenvectors,f,phase,T_max);
W = [];
for i=1:length(free_p)
    %         p = pose+cdpr_v.underactuated_platform.analitic_orthogonal*dpf(:,i);
    %         v = cdpr_v.underactuated_platform.analitic_orthogonal*dvf(:,i);
    %         a = cdpr_v.underactuated_platform.analitic_orthogonal*daf(:,i);
    pose_c = fsolve(@(v) FunDkLFree(cdpr_p,l,free_p(:,i),v),act_pose_0,ut.fsolve_options_grad);
    p = cdpr_p.underactuated_platform.permutation_matrix'*[pose_c;free_p(:,i)];
    cdpr_v = UpdateIKZeroOrd(p(1:3),p(4:end),cdpr_p,cdpr_v);
    
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
        (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
    v = cdpr_v.underactuated_platform.analitic_orthogonal*free_v(:,i);
    cdpr_v = UpdateIKFirstOrd(v(1:3),v(4:end),cdpr_p,cdpr_v);
    
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobiansD...
        (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian_d,cdpr_v.D_mat,cdpr_v.D_mat_d);
    a = cdpr_v.underactuated_platform.analitic_orthogonal_d*free_v(:,i)+...
        cdpr_v.underactuated_platform.analitic_orthogonal*free_a(:,i);
    cdpr_v = UpdateIKSecondOrd(a(1:3),a(4:end),cdpr_p,cdpr_v);
    
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
    cdpr_v.platform = cdpr_v.platform.UpdateCoriolisMatrix(cdpr_p);
    M_ort = CalcMassMatUnder(cdpr_v);
    C_ort = CalcCoriolisMatUnder(cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    f_ort = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.ext_load;
    W_EE = ComputeIdentificationMatrix(cdpr_p,cdpr_v);
    W = [W;cdpr_v.underactuated_platform.geometric_orthogonal'*W_EE];
end
[U,S,V] = svd(W,'econ');
% X = V(:,10)./V(1,10);
% W_hat = W-S(10,10).*U(:,10)*V(:,10)';
% sigma = S(10,10)/sqrt(length(W)-10);
% C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
% sigma_perc = 100*sqrt(diag(C))./X(2:end);
val = 1/S(9,9)+S(1,1)/S(9,9);

end