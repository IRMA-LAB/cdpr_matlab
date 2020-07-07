function [out, pose] = SolveUnderActGeoStatWSPlanar(cdpr_p, cdpr_v, ut,...
  tau_lim, out, pose, varargin)

if (~isempty(varargin))
  rec = varargin{1};
end

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(...
  cdpr_p.underactuated_platform, 0, pose);
cdpr_v.underactuated_platform.unactuated = fsolve(@(v) FunGsNoCheck(...
  cdpr_p, cdpr_v.underactuated_platform.actuated, v),...
  cdpr_v.underactuated_platform.unactuated, ut.fsolve_options_grad);

pose = cdpr_v.underactuated_platform.RecomposeVars(0,...
  cdpr_p.underactuated_platform);
cdpr_v = UpdateIKZeroOrd(pose(1:3), pose(4:end), cdpr_p, cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v, cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

if (isempty(cdpr_v.tension_vector(cdpr_v.tension_vector > tau_lim(2)))...
    && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector < tau_lim(1))))
  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
  K_matrix = CalcStiffnessMatUnder(cdpr_v);
  [~, p] = chol(K_matrix);
  if (p == 0) % stable equilibrium
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
    M_matrix = CalcMassMatUnder(cdpr_v);
    [eigenvectors, eigenvalues_mat] = eig(K_matrix, M_matrix);
    out.counter = out.counter + 1;
    out.pose(:, out.counter) = pose;
    out.position(:, out.counter) = out.pose(1:3, out.counter);
    out.ang_par(:, out.counter) = out.pose(4:end, out.counter);
    out.rot_mat(:, :, out.counter) = cdpr_v.platform.rot_mat;
    out.nat_freq(:, out.counter) = sqrt(diag(eigenvalues_mat)) ./ (2.*pi);
    out.normal_modes(:, :, out.counter) = eigenvectors;
    out.tension_vector(:, out.counter) = cdpr_v.tension_vector;
  end
end
