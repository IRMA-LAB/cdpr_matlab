function vect = IntegrableFreeDynamics(cdpr_p,cdpr_v,time,free_pose,act_pose_0,l,ut)


pose_c = fsolve(@(v) FunDkLFree(cdpr_p,l,free_pose(1:length(free_pose)/2),v),act_pose_0,ut);
pose = cdpr_p.underactuated_platform.permutation_matrix'*[pose_c;free_pose(1:length(free_pose)/2)];
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobians...
  (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian,cdpr_v.D_mat);
pose_d = cdpr_v.underactuated_platform.analitic_orthogonal*free_pose(length(free_pose)/2+1:end);
cdpr_v = UpdateIKFirstOrd(pose_d(1:3),pose_d(4:end),cdpr_p,cdpr_v);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateJacobiansD...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian_d,cdpr_v.D_mat,cdpr_v.D_mat_d);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
cdpr_v.platform = cdpr_v.platform.UpdateCoriolisMatrix(cdpr_p);
M_ort = CalcMassMatUnder(cdpr_v);
C_ort = CalcCoriolisMatUnder(cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
f_ort = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.ext_load;

% Assignment of the state derivative
vect=[free_pose(length(free_pose)/2+1:end);
      linsolve(-M_ort,C_ort*free_pose(length(free_pose)/2+1:end)-f_ort)];

end