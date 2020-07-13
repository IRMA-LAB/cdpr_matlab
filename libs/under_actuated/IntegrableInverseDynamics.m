function vect = IntegrableInverseDynamics(cdpr_p,cdpr_v,...
    sim_data,time,un_act_vars)

% Trajectory settings
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,sim_data.p_s);
act_start = cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,sim_data.p_f);
act_end = cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables);

%normalizedTime = time/sim_data.dt(index);
normalizedTime = time/sim_data.dt;
act_vars = sim_data.motion_law_function(sim_data,normalizedTime,...
    act_start,act_end);

% Coordinate manipulations
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,act_vars(1,:)',...
    un_act_vars(1:cdpr_p.pose_dim-cdpr_p.n_cables));
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(1,act_vars(2,:)',...
    un_act_vars(cdpr_p.pose_dim-cdpr_p.n_cables+1:end));
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(2,act_vars(3,:)',...
    zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1));
pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
pose_d = cdpr_v.underactuated_platform.RecomposeVars(1,cdpr_p.underactuated_platform);

% Model computations
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(pose_d(1:3),pose_d(4:end),cdpr_p,cdpr_v);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
cdpr_v = CalcTotalLoads(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform =...
  cdpr_v.underactuated_platform.UpdateJacobians(cdpr_p.underactuated_platform,...
  cdpr_v.analitic_jacobian,cdpr_v.D_mat);

M_f = cdpr_v.underactuated_platform.geometric_orthogonal'*...
  cdpr_v.platform.mass_matrix_global*cdpr_v.D_mat*cdpr_p.underactuated_platform.permutation_matrix';
f_f = cdpr_v.underactuated_platform.geometric_orthogonal'*...
  cdpr_v.platform.total_load;

% Assignment of the state derivative
vect(1:cdpr_p.pose_dim-cdpr_p.n_cables,1) = cdpr_v.underactuated_platform.pose_P_d(cdpr_p.n_cables+1:end);
vect(cdpr_p.pose_dim-cdpr_p.n_cables+1:2*(cdpr_p.pose_dim-cdpr_p.n_cables),1) =...
  linsolve(M_f(:,cdpr_p.n_cables+1:end),f_f-M_f(:,1:cdpr_p.n_cables)*cdpr_v.underactuated_platform.pose_P_dd(1:cdpr_p.n_cables));

end