function home = SimulateGoToStartProcedureIdeal(cdpr_p, cdpr_v, start_pose,rec,ut,T)

home.pose = fsolve(@(v) FunDkStat(cdpr_p,T(1).*ones(cdpr_p.n_cables,1),v,rec),...
    start_pose,ut.fsolve_options_grad);
cdpr_v = UpdateIKZeroOrd(home.pose(1:3),...
  home.pose(4:end),cdpr_p,cdpr_v);

home.l = zeros(cdpr_p.n_cables,1);
home.sw = zeros(cdpr_p.n_cables,1);
for i=1:cdpr_p.n_cables
  home.l(i,1) = cdpr_v.cable(i).complete_length;
  home.sw(i,1) = cdpr_v.cable(i).swivel_ang;
end


end