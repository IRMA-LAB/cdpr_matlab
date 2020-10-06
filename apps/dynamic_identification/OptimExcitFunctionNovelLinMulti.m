function val = OptimExcitFunctionNovelLinMulti(cdpr_p,cdpr_v,p0,param,ut)

[~,dim] = size(p0);
l0 = param(1:4*dim,1);
in_cond0 = param(4*dim+1:end,1);
W = [];
for j=1:dim
  l = l0(4*(j-1)+1:4*(j-1)+4);
  in_cond = in_cond0(2*(j-1)+1:2*(j-1)+2);
  pose0 = p0(:,j);
  pose = fsolve(@(v) FunDkGsL(cdpr_p,l,v),pose0,ut.fsolve_options_grad);
  cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
  cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
  cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
  K_tot = CalcStiffnessMat(cdpr_v);
  K_ort = CalcStiffnessMatOrt(cdpr_v.underactuated_platform.geometric_orthogonal,K_tot);
  cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
  M_matrix = CalcMassMatUnder(cdpr_v);
  [eigenvectors,eigenvalues_mat] = eig(K_ort,M_matrix);
  for i=1:length(eigenvalues_mat)
    eigenvectors(:,i) = eigenvectors(:,i)./sqrt((eigenvectors(:,i)'*M_matrix*eigenvectors(:,i)));
  end
  f = sqrt(diag(eigenvalues_mat))./(2*pi);
  if (norm(imag(f))==0)  
    T_max = 2;
    ampl_phase = fsolve(@(v) FindAmplPhaseNovel(eigenvectors,f,v,in_cond),zeros(2*length(f),1),ut.fsolve_options_nopar);
    A = ampl_phase(1:length(ampl_phase)/2);
    phase = ampl_phase(length(ampl_phase)/2+1:end);
    [dpf,dvf,daf] = ComputeExcitation(cdpr_p,A,eigenvectors,f,phase,T_max);

    for i=1:length(dpf)
      p = pose+cdpr_v.underactuated_platform.analitic_orthogonal*dpf(:,i);
      v = cdpr_v.underactuated_platform.analitic_orthogonal*dvf(:,i);
      a = cdpr_v.underactuated_platform.analitic_orthogonal*daf(:,i);

      cdpr_v = UpdateIKZeroOrd(p(1:3),p(4:end),cdpr_p,cdpr_v);
      cdpr_v = UpdateIKFirstOrd(v(1:3),v(4:end),cdpr_p,cdpr_v);
      cdpr_v = UpdateIKSecondOrd(a(1:3),a(4:end),cdpr_p,cdpr_v);
    
      W_EE = ComputeIdentificationMatrix(cdpr_p,cdpr_v);
      W = [W;cdpr_v.underactuated_platform.geometric_orthogonal'*W_EE];
    end
  end
end

try
[U,S,V] = svd(W,'econ');
X = V(:,10)./V(1,10);
W_hat = W-S(10,10).*U(:,10)*V(:,10)';
sigma = S(10,10)/sqrt(length(W)-10);
C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
sigma_perc = 100*sqrt(diag(C))./X(2:end);
%val = S(1,1)/S(9,9)+max(abs(sigma_perc))/min(abs(sigma_perc))+max(abs(sigma_perc))+1/S(9,9);
val = S(1,1)/S(9,9)+1/S(9,9);
%val = max(abs(sigma_perc))+max(abs(sigma_perc))/min(abs(sigma_perc));
if (isinf(val) || isnan(val))
    val = 1e56;
end

catch
   val = 1e56;
end

end