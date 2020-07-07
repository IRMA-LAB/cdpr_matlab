function out = GenerateOutputUnderActuated(ind,cdpr_p,cdpr_v,sim_data,geom_fun,ut)

%   timeSpan = 0:ut.t_interval:sim_data.T+2*sim_data.shaper(2,end,ind);
timeSpan = 0:ut.t_interval:6;
  start_coordinate = [sim_data.p(cdpr_p.underactuated_platform.unactuated_mask,ind);zeros(length(cdpr_p.underactuated_platform.unactuated_mask),1)];
  
  sol = RKSolver(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,ind,time,orientation,geom_fun),...
    timeSpan,start_coordinate);
  out = struct();
  for i=1:length(timeSpan)
    
    pStart = sim_data.p(cdpr_p.underactuated_platform.actuated_mask,ind);
    pEnd = sim_data.p(cdpr_p.underactuated_platform.actuated_mask,ind+1);
    normalizedTime = timeSpan(i)/sim_data.dt(ind);
    dofs = sim_data.motion_law_function(sim_data,normalizedTime,...
    geom_fun,pStart,pEnd,sim_data.dt(ind),ind);

    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,dofs(1,:)',...
    sol.y(1:cdpr_p.pose_dim-cdpr_p.n_cables,i));
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(1,dofs(2,:)',...
    sol.y(cdpr_p.pose_dim-cdpr_p.n_cables+1:end,i));
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(2,dofs(3,:)',...
    sol.f(1:cdpr_p.pose_dim-cdpr_p.n_cables,i));
    pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
    pose_d = cdpr_v.underactuated_platform.RecomposeVars(1,cdpr_p.underactuated_platform);
    pose_d_2 = cdpr_v.underactuated_platform.RecomposeVars(2,cdpr_p.underactuated_platform);


    cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(pose_d(1:3),pose_d(4:end),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(pose_d_2(1:3),pose_d_2(4:end),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    out = GetInfoCdpr(cdpr_v,timeSpan(i),i,out);
    
  end

end