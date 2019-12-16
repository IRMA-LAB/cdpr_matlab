function out = GenerateOutputUnderActuated(ind,cdpr_p,cdpr_v,sim_data,geom_fun,ut)

  timeSpan = 0:ut.t_interval:sim_data.T+2*sim_data.shaper(2,end);
  start_coordinate = [sim_data.p(4:6,ind);0;0;0];
  
  sol = RKSolver(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,ind,time,orientation,geom_fun),...
    timeSpan,start_coordinate);
  out = struct();
  for i=1:length(timeSpan)
    
    pStart = sim_data.p(1:cdpr_p.n_cables,ind);
    pEnd = sim_data.p(1:cdpr_p.n_cables,ind+1);
    normalizedTime = timeSpan(i)/sim_data.dt(ind);
    dofs = sim_data.motion_law_function(sim_data,normalizedTime,...
    geom_fun,pStart,pEnd,sim_data.dt(ind));
    cdpr_v = UpdateIKZeroOrd(dofs(1,:)',sol.y(1:3,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dofs(2,:)',sol.y(4:6,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(dofs(3,:)',sol.f(1:3,i),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    out = GetInfoCdpr(cdpr_v,timeSpan(i),i,out);
    
  end

end