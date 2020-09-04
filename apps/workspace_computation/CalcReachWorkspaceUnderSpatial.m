function out = CalcReachWorkspaceUnderSpatial(cdpr_p,cdpr_v,ut,ws_info,rec)

out.counter = 0;
lim = DetermineLimits(cdpr_p,ws_info.z_inferior_limit);


ws_limits = [lim.xyz_mean-lim.dl_frame+lim.dl_plat lim.xyz_mean+lim.dl_frame-lim.dl_plat];
ws_limits(3,2) = ws_limits(3,2)-ws_info.delta_z_safe;

ws_limits = [ws_limits;-pi/18 pi/18; -pi/18 pi/18; -pi/18 pi/18];
if mod(ws_info.mesh_divider,2)==0
  ws_info.mesh_divider = ws_info.mesh_divider-1;
end
ws_info.e_mesh = (ws_info.mesh_divider-1)/2;

iter = zeros(cdpr_p.n_cables,1); iter(cdpr_p.n_cables) = -1;
dir = ones(cdpr_p.n_cables,1);
i = cdpr_p.n_cables;
res = [];

ctr = (ws_limits(:,2)+ws_limits(:,1))/2;
del = (ws_limits(:,2)-ws_limits(:,1))/(ws_info.mesh_divider-1);
del = cdpr_p.underactuated_platform.permutation_matrix*del;
del(cdpr_p.n_cables+1:end) = [];

ws_info.limits = ws_limits;
ctr_p = cdpr_p.underactuated_platform.permutation_matrix*ctr;
contr_ctr = ctr_p(1:cdpr_p.n_cables);
contr = contr_ctr;
free = ctr_p(cdpr_p.n_cables+1:end);
ig_s = ones(cdpr_p.pose_dim-cdpr_p.n_cables,cdpr_p.n_cables).*free;

while 1
  iter(i) = iter(i)+1;
  contr(i) = contr_ctr(i)+dir(i)*iter(i)*del(i);
  if iter(i)> ws_info.e_mesh
    free = ig_s(:,i);
    if dir(i) == 1
      dir(i) = -1;
      iter(i) = 0;
    else
      dir(i) = 1;
      iter(i) = -1;
      i = i-1;
      if i==0
        break
      end
    end
  else
    if i==cdpr_p.n_cables
      conf = cdpr_p.underactuated_platform.permutation_matrix'*[contr;free];
      if ws_info.visualize_cdpr
        [conf,out] = Solve_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out,rec);
      else
        [conf,out] = Solve_GS_WS(cdpr_p,cdpr_v,ut,conf,ws_info.tension_limits,ws_info,out);
      end
      conf_P = cdpr_p.underactuated_platform.permutation_matrix*conf;
      free = conf_P(cdpr_p.n_cables+1:end);
      if (iter(i)==0 && dir(i)==1)
        ig_s(:,i) = free;
      end
    else
      if (iter(i)==1 && dir(i)==1)
        ig_s(:,i) = ig_s(:,i+1);
      end
      i = i+1;
    end
  end
end

end