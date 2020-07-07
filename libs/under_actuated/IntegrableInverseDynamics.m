function vect = IntegrableInverseDynamics(cdpr_p,cdpr_v,...
    sim_data,index,time,un_act_vars,geom_fun)

% Trajectory settings
pStart = sim_data.p(cdpr_p.underactuated_platform.actuated_mask,index);
pEnd = sim_data.p(cdpr_p.underactuated_platform.actuated_mask,index+1);
%normalizedTime = time/sim_data.dt(index);
normalizedTime = time/sim_data.dt(index);
act_vars = sim_data.motion_law_function(sim_data,normalizedTime,...
    geom_fun,pStart,pEnd,sim_data.dt(index),index);

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
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateDynamicsStateSpace(cdpr_p.underactuated_platform,...
    cdpr_v.analitic_jacobian,cdpr_v.platform.mass_matrix_global_ss,cdpr_v.platform.total_load_ss);
cdpr_v = UnderactuatedDynamicsResolution(cdpr_v,cdpr_p.underactuated_platform);

% Assignment of the state derivative
vect(1:cdpr_p.pose_dim-cdpr_p.n_cables,1) = cdpr_v.underactuated_platform.unactuated_deriv;
vect(cdpr_p.pose_dim-cdpr_p.n_cables+1:2*(cdpr_p.pose_dim-cdpr_p.n_cables),1) = cdpr_v.underactuated_platform.unactuated_deriv_2;

end