function out = CalcReachWorkspaceUnderSpatial(cdpr_p,cdpr_v,ut,ws_info,rec)

out.counter = 0;
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);


ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat lim.xyz_mean+lim.dl_frame-lim.dl_plat];
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe;

for x = ws_limits(1,1):(ws_limits(1,2)-ws_limits(1,1))/ws_info.mesh_divider:ws_limits(1,2)
  for y = ws_limits(2,1):(ws_limits(2,2)-ws_limits(2,1))/ws_info.mesh_divider:ws_limits(2,2)
    for z = ws_limits(3,1):(ws_limits(3,2)-ws_limits(3,1))/ws_info.mesh_divider:ws_limits(3,2)
      pose = [x;y;z;zeros(cdpr_p.pose_dim-3,1)];
      out = CalcReachWorkspaceUnderSpatialStep(cdpr_p,cdpr_v,ut,ws_info,rec,out,pose);
    end
  end
end

end