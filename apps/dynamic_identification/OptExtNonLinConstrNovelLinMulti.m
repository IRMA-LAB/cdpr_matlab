function [constr,constreq] = OptExtNonLinConstrNovelLinMulti(cdpr_p,cdpr_v,p0,param,ut)

[~,dim] = size(p0);
l0 = param(1:4*dim,1);
in_cond0 = param(4*dim+1:end,1);
W = [];
tau = [];
In_p = cdpr_p.platform.inertia_mat_G_loc-cdpr_p.platform.mass*...
    Anti(cdpr_p.platform.pos_PG_loc)*Anti(cdpr_p.platform.pos_PG_loc);
par_nominal = [cdpr_p.platform.mass;cdpr_p.platform.pos_PG_loc;In_p(1,1);...
    In_p(2,2); In_p(3,3);In_p(1,2); In_p(1,3);In_p(2,3)];
  
for j=1:dim
  l = l0(4*(j-1)+1:4*(j-1)+4);
  in_cond = in_cond0(2*(j-1)+1:2*(j-1)+2);
  pose0 = p0(:,j);
  pose = fsolve(@(v) FunDkGsL(cdpr_p,l,v),pose0,ut.fsolve_options_grad);
  cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
  cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
  cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
  tau_stat = cdpr_v.tension_vector;
  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
  K_tot = CalcStiffnessMat(cdpr_v);
  K_ort = CalcStiffnessMatOrt(cdpr_v.underactuated_platform.geometric_orthogonal,K_tot);
  cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
  M_matrix = CalcMassMatUnder(cdpr_v);
  K_tau = -cdpr_v.underactuated_platform.geometric_parallel'*K_tot*cdpr_v.underactuated_platform.geometric_orthogonal;
  M_tau = -cdpr_v.underactuated_platform.geometric_parallel'*cdpr_v.platform.mass_matrix_global*cdpr_v.underactuated_platform.geometric_orthogonal;
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
    [dpf,~,daf] = ComputeExcitation(cdpr_p,A,eigenvectors,f,phase,T_max);

    for i=1:length(dpf)
      tau = [tau;tau_stat+(M_tau*daf(:,i)+K_tau*dpf(:,i))];
    end
  else
     tau = [tau;-100000.*ones(100.*cdpr_p.n_cables,1)];
  end
end

constr = -tau;
constreq = 0;

end