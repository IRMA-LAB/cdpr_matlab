function [out,flags] = Calc_ws_flag_u(cdpr_p,cdpr_v,pose,tau_lim,l_err,out,varargin)

if (~isempty(varargin{1}))
  vars = varargin{1};
  rec = vars{1};
end
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);
if(isempty(cdpr_v.tension_vector(cdpr_v.tension_vector>tau_lim(2)))...
    && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<tau_lim(1))))
  flags.tension = 1;
else
  flags.tension = 0;
end
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
  (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
K_tot = CalcStiffnessMat(cdpr_v);
K_ort = CalcStiffnessMatOrt(cdpr_v.underactuated_platform.geometric_orthogonal,K_tot);
d = eig(K_ort);
flags.stable = all(d > 0);

K_ort_l = CalcStiffnessMatOrtL(cdpr_v.underactuated_platform.geometric_orthogonal,...
  cdpr_v.underactuated_platform.geometric_parallel,K_tot);
dz_dl=-cdpr_v.underactuated_platform.analitic_orthogonal*linsolve(K_ort,K_ort_l)+cdpr_v.underactuated_platform.analitic_parallel;
dzP_dl = cdpr_p.underactuated_platform.permutation_matrix*dz_dl;
dzC_dl = dzP_dl(1:cdpr_p.n_cables,1:cdpr_p.n_cables);
if (abs(det(dzC_dl))<0.001)
  flags.singular= 0;
else
  flags.singular = 1;
end

dtau_dl = -cdpr_v.underactuated_platform.geometric_parallel'*K_tot*...
  (-cdpr_v.underactuated_platform.geometric_orthogonal*...
  linsolve(K_ort,cdpr_v.underactuated_platform.geometric_orthogonal')*K_tot...
  +eye(6))*cdpr_v.underactuated_platform.geometric_parallel;

tau_delta = cdpr_v.tension_vector-abs(dtau_dl)*ones(cdpr_p.n_cables,1).*l_err;

if(isempty(tau_delta(tau_delta>tau_lim(2)))...
    && isempty(tau_delta(tau_delta<tau_lim(1))))
  flags.tension2 = 1;
else
  flags.tension2 = 0;
end

if (flags.singular && flags.tension && flags.tension2 && flags.stable) % stable equilibrium
  if (~isempty(varargin{1}))
    rec.SetFrame(cdpr_v,cdpr_p);
  end
  cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
  M_matrix = CalcMassMatUnder(cdpr_v);
  [eigenvectors,eigenvalues_mat] = eig(K_ort,M_matrix);
  out.counter = out.counter+1;
  out.pose(:,out.counter) = pose;
  out.position(:,out.counter) = out.pose(1:3,out.counter);
  out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
  out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
  out.nat_freq(:,out.counter) = sqrt(diag(eigenvalues_mat))./(2.*pi);
  out.normal_modes(:,:,out.counter) = eigenvectors;
  out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
  out.WS_perf(out.counter) = norm(dtau_dl,Inf);
end

end