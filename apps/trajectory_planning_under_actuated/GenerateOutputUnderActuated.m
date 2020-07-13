function out = GenerateOutputUnderActuated(cdpr_p,cdpr_v,sim_data,ut)

%   timeSpan = 0:ut.t_interval:sim_data.T+2*sim_data.shaper(2,end,ind);
timeSpan = 0:ut.t_interval:4;
  cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,sim_data.p_s);
  start_coord = cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end);
  start_coord = [start_coord;zeros(length(start_coord),1)];
  
  sol = RKSolver(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,time,orientation),...
    timeSpan,start_coord);
  out = struct();
  for i=1:length(timeSpan)
    
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,sim_data.p_s);
    act_start = cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables);
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,sim_data.p_f);
    act_end = cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables);
    normalizedTime = timeSpan(i)/sim_data.dt;
    act_vars = sim_data.motion_law_function(sim_data,normalizedTime,...
      act_start,act_end);

    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,act_vars(1,:)',...
    sol.y(1:cdpr_p.pose_dim-cdpr_p.n_cables,i));
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(1,act_vars(2,:)',...
    sol.y(cdpr_p.pose_dim-cdpr_p.n_cables+1:end,i));
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(2,act_vars(3,:)',...
    sol.f(1:cdpr_p.pose_dim-cdpr_p.n_cables,i));
    pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
    pose_d = cdpr_v.underactuated_platform.RecomposeVars(1,cdpr_p.underactuated_platform);
    pose_d_2 = cdpr_v.underactuated_platform.RecomposeVars(2,cdpr_p.underactuated_platform);


    cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(pose_d(1:3),pose_d(4:end),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(pose_d_2(1:3),pose_d_2(4:end),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
      cdpr_v = CalcTotalLoads(cdpr_v,cdpr_p);

    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    out = GetInfoCdpr(cdpr_v,timeSpan(i),i,out);
    
  end

end