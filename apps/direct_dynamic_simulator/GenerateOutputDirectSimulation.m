function out = GenerateOutputDirectSimulation(cdpr_p,cdpr_v,dir_out,out)
  
  for i=1:length(dir_out.t)
    
    cdpr_v = UpdateIKZeroOrd(dir_out.y(1:3,i),dir_out.y(4:cdpr_p.pose_dim,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dir_out.y(cdpr_p.pose_dim+1:cdpr_p.pose_dim+3,i),dir_out.y(cdpr_p.pose_dim+4:end,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(dir_out.f(1:3,i),dir_out.f(4:cdpr_p.pose_dim,i),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    out = GetInfoCdpr(cdpr_v,dir_out.t(i),i,out);
    
  end

end